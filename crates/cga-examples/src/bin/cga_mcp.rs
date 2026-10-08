//! cga_mcp — cga 的 MCP（Model Context Protocol）stdio 服务器。
//!
//! U2（docs/ue58-inspirations.md §3.U2）：把场景会话暴露为 LLM 可调用的工具。
//! 设计要点：
//! - 传输 = stdio，一行一个 JSON-RPC 消息（MCP stdio 约定）；协议子集手写
//!   （initialize / ping / tools/list / tools/call），不引 MCP SDK——协议面
//!   很小，零新框架依赖。
//! - 会话有状态：先 `scene_open`（JSX+CSS 源码直接传入，纯文本是 LLM 友好的
//!   形态），之后全部工具作用于当前会话。
//! - **每个动作都有机器可检查的反馈**：渲染回 PNG（image content），拾取/
//!   碰撞/质量/活动度回结构化 JSON，拖拽回求解到的 q 与残差。
//! - cga 的 Err（不支持/不收敛/不可达/越限）映射为 `isError: true` 的工具
//!   结果，**错误原文透传给 LLM**——绝不假装成功。

use cga_host::*;
use serde_json::{json, Value};
use std::io::{BufRead, Write};

// ---- base64（手写，免依赖）--------------------------------------------------

fn b64(data: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = *chunk.get(1).unwrap_or(&0) as u32;
        let b2 = *chunk.get(2).unwrap_or(&0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        out.push(T[(n >> 18) as usize & 63] as char);
        out.push(T[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 {
            T[(n >> 6) as usize & 63] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            T[n as usize & 63] as char
        } else {
            '='
        });
    }
    out
}

// ---- JSON-RPC 骨架 ----------------------------------------------------------

fn rpc_result(id: &Value, r: Value) -> String {
    json!({"jsonrpc": "2.0", "id": id, "result": r}).to_string()
}

fn rpc_error(id: &Value, code: i64, msg: &str) -> String {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": msg}}).to_string()
}

fn text_block(s: impl Into<String>) -> Value {
    json!({"type": "text", "text": s.into()})
}

/// 工具结果：content blocks。cga 的 Err 一律走 isError（原文透传），不走
/// 协议层 error——LLM 需要读到"为什么不行"。
fn tool_ok(blocks: Vec<Value>) -> Value {
    json!({"content": blocks, "isError": false})
}

fn tool_err(msg: String) -> Value {
    json!({"content": [text_block(msg)], "isError": true})
}

// ---- 工具定义（tools/list）--------------------------------------------------

fn tool_defs() -> Value {
    let schema = |props: Value, required: &[&str]| json!({"type": "object", "properties": props, "required": required});
    let vec3 =
        || json!({"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3});
    json!([
        {
            "name": "scene_open",
            "description": "打开一个 JSX+CSS 场景会话（纯文本源码直接传入）。之后的所有工具作用于当前会话。返回场景摘要（对象/连杆/运动副/活动度）。",
            "inputSchema": schema(json!({
                "jsx": {"type": "string", "description": "JSX 场景源码（export default (...))"},
                "css": {"type": "string", "description": "可选 CSS 源码"},
                "pose": {"type": "object", "description": "可选 pose 覆盖 {pair 名: q}（1-DOF 副）"},
            }), &["jsx"]),
        },
        {
            "name": "scene_close",
            "description": "关闭当前场景会话。",
            "inputSchema": schema(json!({}), &[]),
        },
        {
            "name": "scene_report",
            "description": "当前场景的完整文本报告（相机/灯光/对象/材质/运动学/质量属性）。",
            "inputSchema": schema(json!({}), &[]),
        },
        {
            "name": "scene_render",
            "description": "渲染当前场景，返回 PNG 图像。用于目视自查：建模后渲染检查是标准闭环。channel 参数切诊断通道（headless 的视口：对象 id 调色板 / 法线 / 深度）。",
            "inputSchema": schema(json!({
                "w": {"type": "integer", "default": 640},
                "h": {"type": "integer", "default": 480},
                "aa": {"type": "integer", "default": 2, "description": "每像素采样数（抗锯齿）"},
                "opaque": {"type": "boolean", "default": false, "description": "忽略透明度（无反射/折射）"},
                "channel": {"type": "string", "enum": ["id", "normals", "depth"], "description": "诊断通道（给出时忽略 aa/opaque）"},
            }), &[]),
        },
        {
            "name": "scene_pick",
            "description": "像素拾取：从像素 (x,y) 发一条光线，返回最近命中的对象/点/法线/所属 link。坐标系：像素，左上角原点。",
            "inputSchema": schema(json!({
                "x": {"type": "number"}, "y": {"type": "number"},
                "w": {"type": "integer"}, "h": {"type": "integer"},
            }), &["x", "y", "w", "h"]),
        },
        {
            "name": "scene_drag",
            "description": "抓取拖拽：把 link 上当前位于世界点 from 的点拖到世界点 to，解路径上自由 1-DOF 运动副的 q 并写回场景。不可达/不收敛/越限/含 cam/gear/闭链时返回错误（场景不变）。",
            "inputSchema": schema(json!({
                "link": {"type": "string"},
                "from": vec3(),
                "to": vec3(),
            }), &["link", "from", "to"]),
        },
        {
            "name": "scene_drag_pose",
            "description": "位姿抓取：把 link 的 frame 拖到目标位姿（4×4 齐次矩阵，行主序 16 个数，旋转部分必须正规）。残差 = 位置差 + rotvec。边界同 scene_drag。",
            "inputSchema": schema(json!({
                "link": {"type": "string"},
                "target": {"type": "array", "items": {"type": "number"}, "minItems": 16, "maxItems": 16},
            }), &["link", "target"]),
        },
        {
            "name": "scene_collisions",
            "description": "全场景碰撞扫描（认证三值判定：Yes/No/Unknown，Unknown 绝不假装精确）。返回命中对列表与缓存统计。",
            "inputSchema": schema(json!({}), &[]),
        },
        {
            "name": "scene_mass",
            "description": "各对象质量属性（density prop 链上的对象才有；平面/圆片等无有限体积的图元诚实跳过）。",
            "inputSchema": schema(json!({}), &[]),
        },
        {
            "name": "scene_mobility",
            "description": "机构活动度报告（Grübler 口径：总自由度/作者钉/gear/cam/闭链/净活动度）。",
            "inputSchema": schema(json!({}), &[]),
        },
    ])
}

// ---- 服务器 -----------------------------------------------------------------

struct Server {
    sess: Option<SceneSession>,
}

impl Server {
    fn sess(&mut self) -> Result<&mut SceneSession, String> {
        self.sess
            .as_mut()
            .ok_or_else(|| "还没有场景：先 scene_open".to_string())
    }

    fn call(&mut self, name: &str, args: &Value) -> Result<Value, String> {
        match name {
            "scene_open" => self.scene_open(args),
            "scene_close" => {
                self.sess = None;
                Ok(tool_ok(vec![text_block("closed")]))
            }
            "scene_report" => {
                let run = self.sess()?.run();
                Ok(tool_ok(vec![text_block(scene_report::scene_report(
                    &run.scene,
                    &run.camera,
                    &run.tags,
                    &run.kinematics,
                    &run.mass_props,
                ))]))
            }
            "scene_render" => self.scene_render(args),
            "scene_pick" => self.scene_pick(args),
            "scene_drag" => self.scene_drag(args),
            "scene_drag_pose" => self.scene_drag_pose(args),
            "scene_collisions" => self.scene_collisions(),
            "scene_mass" => self.scene_mass(),
            "scene_mobility" => self.scene_mobility(),
            _ => Err(format!("未知工具 {name}")),
        }
    }

    fn scene_open(&mut self, args: &Value) -> Result<Value, String> {
        let jsx = args
            .get("jsx")
            .and_then(Value::as_str)
            .ok_or("scene_open 需要 jsx 参数")?;
        let css = args.get("css").and_then(Value::as_str);
        let pose: Vec<(String, f64)> = match args.get("pose") {
            Some(Value::Object(m)) => m
                .iter()
                .map(|(k, v)| {
                    v.as_f64()
                        .map(|x| (k.clone(), x))
                        .ok_or_else(|| format!("pose.{k} 不是数字"))
                })
                .collect::<Result<_, _>>()?,
            _ => Vec::new(),
        };
        let sess = SceneSession::open_pose(jsx, css, ".", &pose)?;
        let summary = {
            let run = sess.run();
            let mob = mobility(&run.kinematics);
            let links: Vec<&str> = run
                .kinematics
                .links
                .iter()
                .map(|l| l.name.as_str())
                .collect();
            let pairs: Vec<String> = run
                .kinematics
                .pairs
                .iter()
                .map(|p| {
                    format!(
                        "{}({})",
                        p.name.as_deref().unwrap_or("<unnamed>"),
                        p.kind.name()
                    )
                })
                .collect();
            format!(
                "scene opened: {} objects, links {links:?}, pairs {pairs:?}, mobility dof={} (gross={} pins={} gears={} cams={} closures={})",
                run.scene.objects.len(),
                mob.dof,
                mob.gross_dofs,
                mob.author_pins,
                mob.gears,
                mob.cams,
                mob.closure_pins,
            )
        };
        self.sess = Some(sess);
        Ok(tool_ok(vec![text_block(summary)]))
    }

    fn scene_render(&mut self, args: &Value) -> Result<Value, String> {
        let w = args.get("w").and_then(Value::as_i64).unwrap_or(640) as i32;
        let h = args.get("h").and_then(Value::as_i64).unwrap_or(480) as i32;
        let aa = args.get("aa").and_then(Value::as_i64).unwrap_or(2) as i32;
        // 诊断通道（U3）：id / normals / depth——headless 引擎的"视口"。
        if let Some(ch) = args.get("channel").and_then(Value::as_str) {
            let channel = match ch {
                "id" => cga_gpu::DiagChannel::ObjectId,
                "normals" => cga_gpu::DiagChannel::Normal,
                "depth" => cga_gpu::DiagChannel::Depth,
                _ => return Err(format!("未知 channel {ch}（可选 id / normals / depth）")),
            };
            let img = self.sess()?.render_diagnostic(w, h, channel)?;
            return Ok(tool_ok(vec![
                json!({"type": "image", "data": b64(&img.png), "mimeType": "image/png"}),
                text_block(format!("diagnostic {w}x{h} channel={ch}")),
            ]));
        }
        let mode = if args.get("opaque").and_then(Value::as_bool).unwrap_or(false) {
            RenderMode::IgnoreOpacity
        } else {
            RenderMode::Normal
        };
        let (img, stats) = self.sess()?.render_incremental(w, h, aa, mode)?;
        Ok(tool_ok(vec![
            json!({"type": "image", "data": b64(&img.png), "mimeType": "image/png"}),
            text_block(format!(
                "rendered {w}x{h} aa={aa} mode={mode:?}; incremental: full={} reason={} rays={}/{}",
                stats.full, stats.reason, stats.dirty, stats.total
            )),
        ]))
    }

    fn scene_pick(&mut self, args: &Value) -> Result<Value, String> {
        let num = |k: &str| {
            args.get(k)
                .and_then(Value::as_f64)
                .ok_or_else(|| format!("scene_pick 需要数字参数 {k}"))
        };
        let (x, y) = (num("x")?, num("y")?);
        let (w, h) = (num("w")? as i32, num("h")? as i32);
        let sess = self.sess()?;
        let Some((hit, inst)) = sess.pick(x, y, w, h) else {
            return Ok(tool_ok(vec![text_block("no hit")]));
        };
        let link = sess
            .run()
            .kinematics
            .links
            .iter()
            .find(|l| l.meshes.contains(&hit.object))
            .map(|l| l.name.clone());
        Ok(tool_ok(vec![text_block(
            json!({
                "object": hit.object,
                "point": hit.point,
                "normal": hit.normal,
                "t": hit.t,
                "instance": inst,
                "link": link,
            })
            .to_string(),
        )]))
    }

    fn scene_drag(&mut self, args: &Value) -> Result<Value, String> {
        let link = args
            .get("link")
            .and_then(Value::as_str)
            .ok_or("scene_drag 需要 link")?;
        let v3 = |k: &str| -> Result<[f64; 3], String> {
            let v = args
                .get(k)
                .and_then(Value::as_array)
                .ok_or_else(|| format!("scene_drag 需要数组参数 {k}"))?;
            if v.len() != 3 {
                return Err(format!("scene_drag 参数 {k} 需要 3 个数"));
            }
            let mut out = [0.0; 3];
            for (i, x) in v.iter().enumerate() {
                out[i] = x
                    .as_f64()
                    .ok_or_else(|| format!("scene_drag 参数 {k}[{i}] 不是数字"))?;
            }
            Ok(out)
        };
        let d = self.sess()?.drag(link, v3("from")?, v3("to")?)?;
        Ok(tool_ok(vec![text_block(
            json!({
                "link": d.link,
                "solved": d.solved,
                "grab": d.grab,
                "residual": d.residual,
            })
            .to_string(),
        )]))
    }

    fn scene_drag_pose(&mut self, args: &Value) -> Result<Value, String> {
        let link = args
            .get("link")
            .and_then(Value::as_str)
            .ok_or("scene_drag_pose 需要 link")?;
        let v = args
            .get("target")
            .and_then(Value::as_array)
            .ok_or("scene_drag_pose 需要 target 数组")?;
        if v.len() != 16 {
            return Err("scene_drag_pose 的 target 需要 16 个数（行主序 4×4）".to_string());
        }
        let mut target = [0.0; 16];
        for (i, x) in v.iter().enumerate() {
            target[i] = x.as_f64().ok_or_else(|| format!("target[{i}] 不是数字"))?;
        }
        let d = self.sess()?.drag_pose(link, target)?;
        Ok(tool_ok(vec![text_block(
            json!({
                "link": d.link,
                "solved": d.solved,
                "pose": d.pose,
                "residual": d.residual,
            })
            .to_string(),
        )]))
    }

    fn scene_collisions(&mut self) -> Result<Value, String> {
        let sess = self.sess()?;
        let hits = sess.collisions();
        let (cache_hits, computed) = sess.collision_scan_stats();
        let list: Vec<Value> = hits
            .iter()
            .map(|p| {
                json!({
                    "a": p.a,
                    "b": p.b,
                    "hit": format!("{:?}", p.hit),
                    "separation": p.separation,
                })
            })
            .collect();
        Ok(tool_ok(vec![text_block(
            json!({"pairs": list, "cache_hits": cache_hits, "narrowphase_computed": computed})
                .to_string(),
        )]))
    }

    fn scene_mass(&mut self) -> Result<Value, String> {
        let sess = self.sess()?;
        let run = sess.run();
        let mut total = 0.0;
        let list: Vec<Value> = run
            .mass_props
            .iter()
            .enumerate()
            .filter_map(|(i, mp)| {
                mp.as_ref().map(|m| {
                    total += m.mass;
                    json!({"object": i, "mass": m.mass, "cog": m.cog})
                })
            })
            .collect();
        Ok(tool_ok(vec![text_block(
            json!({"objects": list, "total_mass": total}).to_string(),
        )]))
    }

    fn scene_mobility(&mut self) -> Result<Value, String> {
        let mob = mobility(&self.sess()?.run().kinematics);
        Ok(tool_ok(vec![text_block(
            json!({
                "gross_dofs": mob.gross_dofs,
                "author_pins": mob.author_pins,
                "gears": mob.gears,
                "cams": mob.cams,
                "closure_pins": mob.closure_pins,
                "dof": mob.dof,
            })
            .to_string(),
        )]))
    }
}

// ---- stdio 主循环 -----------------------------------------------------------

fn main() {
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    let mut srv = Server { sess: None };
    for line in stdin.lock().lines() {
        let Ok(line) = line else { break };
        if line.trim().is_empty() {
            continue;
        }
        let msg: Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(e) => {
                let _ = writeln!(
                    out,
                    "{}",
                    rpc_error(&Value::Null, -32700, &format!("parse error: {e}"))
                );
                let _ = out.flush();
                continue;
            }
        };
        let method = msg.get("method").and_then(Value::as_str).unwrap_or("");
        let id = msg.get("id").cloned();
        let Some(id) = id else {
            continue; // notification（initialized / exit 等）：不需要响应
        };
        let response = match method {
            "initialize" => {
                let pv = msg
                    .pointer("/params/protocolVersion")
                    .and_then(Value::as_str)
                    .unwrap_or("2025-06-18");
                rpc_result(
                    &id,
                    json!({
                        "protocolVersion": pv,
                        "capabilities": {"tools": {"listChanged": false}},
                        "serverInfo": {"name": "cga-mcp", "version": env!("CARGO_PKG_VERSION")},
                    }),
                )
            }
            "ping" => rpc_result(&id, json!({})),
            "tools/list" => rpc_result(&id, json!({"tools": tool_defs()})),
            "tools/call" => {
                let name = msg
                    .pointer("/params/name")
                    .and_then(Value::as_str)
                    .unwrap_or("");
                let args = msg
                    .pointer("/params/arguments")
                    .cloned()
                    .unwrap_or(json!({}));
                let r = srv.call(name, &args);
                rpc_result(&id, r.unwrap_or_else(tool_err))
            }
            _ => rpc_error(&id, -32601, &format!("method not found: {method}")),
        };
        let _ = writeln!(out, "{response}");
        let _ = out.flush();
    }
}
