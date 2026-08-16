from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import mlx.core as mx

from cga.engine.ambient_light import AmbientLight
from cga.engine.material import Material

if TYPE_CHECKING:
    from cga.engine.perspective_camera import PerspectiveCamera
    from cga.engine.scene import Scene


class Renderer:
    """离屏光线追踪渲染器 (three.js WebGLRenderer 的 MLX 对应物)。

    render(scene, camera) -> (H, W, 4) uint8 RGBA (GPU→CPU 每帧回传)。
    每帧: 全部图元共轭进相机空间 (CPU, 图元数 ~10 可忽略) → 批量求交
    (MLX GPU, 全像素向量化) → Blinn-Phong 着色 (每光源一条阴影射线,
    透明遮挡物按 1−opacity 透光) → 线性空间累加 → 输出端 sRGB 编码。
    透明面走批量 Whitted 递归 (Fresnel 分裂反射/折射两束, Beer 吸收,
    max_depth 截断)。
    """

    def __init__(
        self, width: int = 640, height: int = 480, aa: int = 1, max_depth: int = 3
    ):
        """超采样抗锯齿: aa=1 每像素 1 条射线 (像素中心, 原行为);
        aa=N → 每像素 N×N 条分层亚像素射线取平均 (SSAA, 射线一次批量)。
        max_depth: 反射/折射递归层数 (仅场景含透明体时才有开销)。"""
        if aa < 1:
            raise ValueError(f"aa must be >= 1, got {aa}")
        if max_depth < 0:
            raise ValueError(f"max_depth must be >= 0, got {max_depth}")
        self.width = int(width)
        self.height = int(height)
        self.aa = int(aa)
        self.max_depth = int(max_depth)
        self.cam = None
        self.rays = None  # (aa²·N, 3) 单位方向, N = H·W, 按相机内参惰性构建

    def build_rays(self) -> mx.array:
        """像素网格 → 相机空间单位射线方向 (相机在原点, 无透视偏移)。

        aa>1: 每像素 k×k (k=aa) 分层亚像素采样, 射线数 = aa²·H·W,
        渲染后按样本平均 (框滤波超采样 = SSAA)。aa=1 与原行为逐位一致。
        """
        H, W = self.height, self.width
        fy = H / (2.0 * math.tan(math.radians(self.cam.fov) / 2.0))
        fx = fy * self.cam.aspect
        cx, cy = (W - 1) / 2.0, (H - 1) / 2.0
        u0 = (mx.arange(W, dtype=mx.float32) - cx) / fx  # (W,) 像素中心
        v0 = (mx.arange(H, dtype=mx.float32) - cy) / fy  # (H,)
        z = mx.ones((H, W), dtype=mx.float32)
        dirs = []
        k = self.aa
        for j in range(k):
            for i in range(k):
                du = ((i + 0.5) / k - 0.5) / fx  # 亚像素偏移 (射线单位, 像素内分层)
                dv = ((j + 0.5) / k - 0.5) / fy
                u = mx.broadcast_to((u0 + du)[None, :], (H, W))
                v = mx.broadcast_to((v0 + dv)[:, None], (H, W))
                dirs.append(mx.stack([u, v, z], axis=-1))
        rays = mx.concatenate(dirs, axis=0).reshape(-1, 3)  # (aa²·N, 3)
        n = mx.sqrt(mx.sum(rays * rays, axis=-1, keepdims=True))
        return rays / n

    def render(self, scene: Scene, camera: PerspectiveCamera) -> mx.array:
        """渲染一帧 → (H, W, 4) uint8 RGBA。"""
        self.cam = camera
        if self.rays is None:
            self.rays = self.build_rays()
        o = mx.zeros_like(self.rays)  # 相机在原点 (N,3)
        N = o.shape[0]
        bg = mx.broadcast_to(mx.array(scene.background.rgb(), dtype=mx.float32), (N, 3))
        # 灯光共轭进相机空间 (多态 to_camera; 环境光独立槽 —— 注册式路由)
        lit = []
        ambient = None
        for light in scene.lights:
            if isinstance(light, AmbientLight):
                ambient = light
            else:
                lit.append(light.to_camera(camera.motor))
        in_medium = mx.zeros((N,), dtype=mx.bool_)
        sigma = mx.zeros((N,), dtype=mx.float32)
        rgb = self.trace(
            scene, camera, o, self.rays, lit, ambient, bg, in_medium, sigma, 0
        )
        S = self.aa * self.aa
        if S > 1:
            # 超采样平均: (aa²·N, 3) → 每像素 aa² 条样本取均值
            rgb = mx.mean(mx.reshape(rgb, (S, N // S, 3)), axis=0)
        # 线性 → sRGB 编码 (颜色在 Color 构造时解码进线性空间, 此处
        # roundtrip 恒等; >1 高光硬截断 —— ponytail: 需要柔和高光滚降时
        # 加一行 Reinhard tonemap: rgb = rgb/(1+rgb))
        rgb = mx.clip(rgb, 0.0, 1.0)
        rgb = mx.where(
            rgb <= 0.0031308,
            12.92 * rgb,
            1.055 * mx.power(rgb, 1.0 / 2.4) - 0.055,
        )
        rgba = mx.concatenate([rgb, mx.ones((N // S, 1), dtype=mx.float32)], axis=-1)
        rgba = mx.clip(rgba * 255.0 + 0.5, 0.0, 255.0).astype(mx.uint8)
        return mx.reshape(rgba, (self.height, self.width, 4))

    def nearest(
        self,
        scene: Scene,
        camera: PerspectiveCamera,
        o: mx.array,
        d: mx.array,
        lit: Sequence,
        ambient: AmbientLight | None,
        primary: bool = False,
    ) -> tuple[mx.array, mx.array, mx.array, mx.array, mx.array, mx.array, mx.array]:
        """一束射线 vs 场景全部图元 → 每像素最近命中 + 局部着色。

        primary=True 仅当射线是相机主射线 (原点在相机、方向 d_z>0): 此时可
        对相机空间 AABB 做保守后方剔除。阴影/递归射线 origin/direction 任意,
        不适用。

        返回 (hit, t, n, col, opacity, ior, absorption); 未命中像素为占位值。

        两遍: 先选最近命中, 再从命中点向各光源发阴影射线 (透明遮挡物按
        (1−opacity) 累乘透光, 忽略 Fresnel —— ponytail: 玻璃阴影是近似),
        最后共享命中点逐材质着色、按最近图元索引选取。
        """
        N = o.shape[0]
        best_t = mx.full((N,), float("inf"), dtype=mx.float32)
        best_n = mx.zeros((N, 3), dtype=mx.float32)
        best_uv = mx.zeros((N, 2), dtype=mx.float32)
        best_idx = mx.zeros((N,), dtype=mx.int32)
        objs = scene.objects
        opacity = mx.ones((N,), dtype=mx.float32)
        iors = mx.full((N,), 1.5, dtype=mx.float32)
        absos = mx.zeros((N,), dtype=mx.float32)
        # 逐对象标量 (opacity/ior/absorption) 预先堆叠, 循环后按最近对象
        # gather —— 避免循环内每对象 3 次全帧 where (对象多时是主要开销)。
        if objs:
            opacity = mx.stack(
                [mx.array(obj.material.opacity, dtype=mx.float32) for obj in objs]
            )
            iors = mx.stack(
                [mx.array(obj.material.ior, dtype=mx.float32) for obj in objs]
            )
            absos = mx.stack(
                [mx.array(obj.material.absorption, dtype=mx.float32) for obj in objs]
            )
        params_list = []
        for i, obj in enumerate(objs):
            wm = camera.motor.compose(obj.motor())  # 世界→相机 · 局部→世界
            params = obj.geometry.to_camera(wm)
            params_list.append(params)
            if primary:
                bnd = obj.geometry.bounds_camera(params)
                if bnd is not None and bnd[1][2] <= 1e-6:
                    continue  # AABB 整体在相机后 → 主射线 (d_z>0) 不可能命中
            t, n_i, mask = obj.geometry.intersect(params, o, d)
            uv_i = obj.geometry.uv_at(params, o + t[:, None] * d, n_i)
            nearer = mx.logical_and(mask, t < best_t)
            best_t = mx.where(nearer, t, best_t)
            best_n = mx.where(nearer[:, None], n_i, best_n)
            best_uv = mx.where(nearer[:, None], uv_i, best_uv)
            best_idx = mx.where(nearer, i, best_idx)
        hit = mx.isfinite(best_t)  # 命中像素 t 有限; 未命中保持 inf
        if objs:
            op = mx.take(opacity, best_idx, axis=0)
            ior = mx.take(iors, best_idx, axis=0)
            abso = mx.take(absos, best_idx, axis=0)
        else:
            op = mx.ones((N,), dtype=mx.float32)
            ior = mx.full((N,), 1.5, dtype=mx.float32)
            abso = mx.zeros((N,), dtype=mx.float32)
        cos_i = -mx.sum(d * best_n, axis=-1, keepdims=True)
        best_n = mx.where(cos_i < 0.0, -best_n, best_n)  # 法向翻向射线起点侧
        p = o + best_t[:, None] * d
        # 阴影射线: 每光源 (从命中点 + eps·n 出发) 的逐像素可见性
        p_s = p + 1e-3 * best_n
        vis = []
        for light in lit:
            ld, _ = light.direction_at(p)
            far = light.far(p)
            v = mx.ones((N,), dtype=mx.float32)
            for obj, params in zip(scene.objects, params_list):
                _t, m = obj.geometry.intersect_shadow(params, p_s, ld)
                occ = m if far is None else mx.logical_and(m, _t < far)
                v = v * mx.where(occ, 1.0 - obj.material.opacity, 1.0)
            vis.append(v)
        if scene.objects:
            # 逐对象标量参数 → 按最近对象 gather 成逐像素参数, 只着色一次。
            # (旧实现: 每个对象在每像素都 shade 一遍再挑选, O(O×L×N) 且
            #  中间张量 (O,N,3) 在对象多时吃满显存带宽。)
            params = [obj.material.shade_params() for obj in scene.objects]
            emissive = mx.stack([prm[0] for prm in params])  # (O,3)
            diff = mx.stack([prm[1] for prm in params])  # (O,3)
            spec = mx.stack([prm[2] for prm in params])  # (O,3)
            expo = mx.stack(
                [mx.array(prm[3], dtype=mx.float32) for prm in params]
            )  # (O,)
            col = Material.shade_batched(
                mx.take(emissive, best_idx, axis=0),  # (N,3)
                mx.take(diff, best_idx, axis=0),
                mx.take(spec, best_idx, axis=0),
                mx.take(expo, best_idx, axis=0)[:, None],  # (N,1)
                p,
                best_n,
                d,
                lit,
                ambient,
                vis,
            )
            # Each texture samples all pixels in one MLX kernel; the nearest
            # object index selects only its owning pixels. This preserves
            # differently sized images without per-pixel Python dispatch.
            for i, obj in enumerate(scene.objects):
                texture = getattr(obj.material, "map", None)
                if texture is not None:
                    sampled = texture.sample(best_uv)[:, :3]
                    col = mx.where((best_idx == i)[:, None], col * sampled, col)
        else:
            col = mx.zeros((N, 3), dtype=mx.float32)
        return hit, best_t, best_n, col, op, ior, abso

    def trace(
        self,
        scene: Scene,
        camera: PerspectiveCamera,
        o: mx.array,
        d: mx.array,
        lit: Sequence,
        ambient: AmbientLight | None,
        bg: mx.array,
        in_medium: mx.array,
        sigma: mx.array,
        depth: int,
    ) -> mx.array:
        """一束射线 (N,3) 的批量 Whitted 追踪 → (N,3) 线性颜色。

        透明命中按精确非偏振 Fresnel F 分裂: F·反射 + (1−F)·(α·本体色 +
        (1−α)·折射)。ior=1 时 F≡0、折射方向不变 → 精确退化为原 alpha
        front-to-back 混合 (旧行为是 F=0 特例)。全反射 (TIR) 时 F=1。
        Beer 吸收: in_medium 像素的结果乘 exp(−sigma·t) (t = 介质内行程;
        未命中 = 逃出场景, 不衰减)。
        ponytail: 介质追踪 = 每像素一个 in_medium 位 + 标量 sigma (假设
        透明体互不重叠、嵌在空气中); 嵌套介质需每像素折射率栈。
        """
        hit, t, n, local, op, ior, abso = self.nearest(
            scene, camera, o, d, lit, ambient, primary=(depth == 0)
        )
        cos_i = -mx.sum(d * n, axis=-1, keepdims=True)
        n = mx.where(cos_i < 0.0, -n, n)  # 防御: 法向一律翻向射线起点侧
        cos_i = mx.abs(cos_i)
        result = mx.where(hit[:, None], local, bg)
        if depth < self.max_depth:
            need = mx.logical_and(hit, op < 1.0)
            eta = mx.where(in_medium[:, None], ior[:, None], 1.0 / ior[:, None])
            k = 1.0 - eta * eta * (1.0 - cos_i * cos_i)
            cos_t = mx.sqrt(mx.maximum(k, 0.0))  # TIR 时占位, F 覆写为 1
            g = 1.0 / eta  # n2/n1
            # 精确非偏振 Fresnel (s/p 偏振平均)
            rs = (cos_i - g * cos_t) / mx.maximum(cos_i + g * cos_t, 1e-12)
            rp = (cos_t - g * cos_i) / mx.maximum(cos_t + g * cos_i, 1e-12)
            fres = 0.5 * (rs * rs + rp * rp)
            fres = mx.where(k <= 0.0, mx.ones_like(fres), fres)  # TIR → 全反射
            eps = 1e-3  # 自相交偏移
            d_r = d + 2.0 * cos_i * n
            d_t = eta * d + (eta * cos_i - cos_t) * n
            # 分支剪枝: 整束权重可忽略时跳过递归。单次同步取
            # (需要递归的像素数, 反射/折射最大权重) 供 Python 分支。
            m_arr = mx.sum(need).astype(mx.float32)
            w_r = mx.max(mx.where(need, fres[:, 0], 0.0))
            w_t = ((1.0 - fres) * (1.0 - op[:, None]))[:, 0]
            w_t = mx.max(mx.where(need, w_t, 0.0))
            m_count, w_r_v, w_t_v = mx.stack([m_arr, w_r, w_t]).tolist()
            m_count = int(m_count)
            if m_count > 0:
                # 只把 need 的透明子集射线压进紧凑数组递归 —— 避免对整帧
                # 重复 nearest/阴影/着色 (旧实现玻璃场景 ≈ 3× 全帧)。
                # 射线计算逐像素独立, 子集与全帧结果逐位一致。
                # MLX 无 nonzero/argwhere/布尔索引, 用 argsort 取 need 的扁平索引。
                idx = mx.argsort(mx.where(need, 0, 1).astype(mx.int32))[:m_count]
                o_c = o[idx]
                d_c = d[idx]
                n_c = n[idx]
                t_c = t[idx]
                p_c = o_c + t_c[:, None] * d_c
                local_c = local[idx]
                op_c = op[idx]
                abso_c = abso[idx]
                in_c = in_medium[idx]
                sigma_c = sigma[idx]
                bg_c = bg[idx]
                fres_c = fres[idx]
                d_r_c = d_r[idx]
                d_t_c = d_t[idx]
                refl = (
                    self.trace(
                        scene,
                        camera,
                        p_c + eps * n_c,
                        d_r_c,
                        lit,
                        ambient,
                        bg_c,
                        in_c,
                        sigma_c,
                        depth + 1,
                    )
                    if w_r_v > 1e-3
                    else mx.zeros((m_count, 3), dtype=mx.float32)
                )
                # 折射进入/离开介质: 更新介质位与吸收系数 (子集上 need 恒真)
                entering_c = mx.logical_not(in_c)
                sig_next_c = mx.where(entering_c, abso_c, 0.0)
                refr = (
                    self.trace(
                        scene,
                        camera,
                        p_c - eps * n_c,
                        d_t_c,
                        lit,
                        ambient,
                        bg_c,
                        mx.logical_not(in_c),
                        sig_next_c,
                        depth + 1,
                    )
                    if w_t_v > 1e-3
                    else mx.zeros((m_count, 3), dtype=mx.float32)
                )
                glass_c = fres_c * refl + (1.0 - fres_c) * (
                    op_c[:, None] * local_c + (1.0 - op_c[:, None]) * refr
                )
                result = mx.put_along_axis(result, idx[:, None], glass_c, axis=0)
        # Beer: 介质内传播衰减
        att = mx.where(
            mx.logical_and(in_medium, hit)[:, None],
            mx.exp(-sigma[:, None] * t[:, None]),
            1.0,
        )
        return result * att

    @staticmethod
    def render_frame(
        scene: Scene,
        camera: PerspectiveCamera,
        width: int = 640,
        height: int = 480,
        aa: int = 1,
    ) -> mx.array:
        """单帧快捷入口 (demo 用)。"""
        return Renderer(width, height, aa=aa).render(scene, camera)

    @staticmethod
    def frame_to_bytes(img: mx.array) -> bytes:
        """(H, W, 4) uint8 RGBA 帧 → 扁平 RGBA bytes (PIL 输出桥)。

        demo 层用 `Image.frombytes("RGBA", (w, h), Renderer.frame_to_bytes(img))`
        存图。纯 MLX: mx.eval 把惰性图落地到主机内存, 再走 buffer 协议
        一次取字节 (C-order RGBA)。旧实现 img.tolist() 逐元素 Python 生成器
        ~67ms; 此路径 ~9ms, 剩余瓶颈是 GPU→CPU 拷贝而非转换方式。
        """
        mx.eval(img)  # 惰性图落地到主机内存 (eval 就地求值, 返回 None)
        return bytes(memoryview(img))
