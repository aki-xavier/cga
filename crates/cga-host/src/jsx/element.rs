//! 元素值层：boa 执行结果（JSON）→ `El` 树；prop 读取器（camelCase ↔
//! snake_case 双写兼容）；`Value` → `ArgValue` 转换与材质 prop 键表。

use std::collections::HashMap;

use serde_json::Value;

use crate::scene_build::ArgValue;

/// boa executes the compiled module; the scene element tree comes back as
/// JSON (all prop values are numbers/strings/bools/arrays/objects).
#[derive(Clone, Debug)]
pub struct El {
    pub tag: String,
    pub props: HashMap<String, Value>,
    pub children: Vec<El>,
    /// 自身提交版本（快照 `__v`）：创建 / props 更新 / 子节点增删移动时递增。
    /// 子树版本见 [`El::s`]。0 = 无版本（嵌套在 props 里的元素树），增量构建
    /// 永不复用无版本子树。
    pub v: u64,
    /// 子树版本（快照 `__s`）：自身与全部后代 `v` 的最大值。
    pub s: u64,
    /// React 宿主实例 id（快照 `__id`；拾取 → 事件派发的映射）。合成节点
    /// （fragment 包装等）没有实例 id。
    pub instance_id: Option<i64>,
    /// 增量构建缓存：本子树最近一帧产出的 `scene.objects` 区间。只在会话缓存的
    /// 上一帧树上填充；新帧由 `annotate_reuse` 标注后，`walk` 据此整棵复用。
    pub cache: Option<std::ops::Range<usize>>,
}

pub(crate) fn to_el(v: &Value) -> Result<El, String> {
    let obj = v
        .as_object()
        .ok_or_else(|| format!("JSX: scene is not an element: {v}"))?;
    let tag = obj.get("t").and_then(Value::as_str).ok_or_else(|| {
        let t = v.to_string();
        let t = if t.len() > 240 {
            format!("{}…", &t[..240])
        } else {
            t
        };
        format!("JSX: element without tag: {t}")
    })?;
    let props: HashMap<String, Value> = obj
        .get("p")
        .and_then(Value::as_object)
        .map(|m| m.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
        .unwrap_or_default();
    let mut children = Vec::new();
    if let Some(c) = obj.get("c").and_then(Value::as_array) {
        for c in c {
            if c.is_null() {
                continue;
            }
            children.push(to_el(c)?);
        }
    }
    let v = obj.get("__v").and_then(Value::as_u64).unwrap_or(0);
    let s = obj.get("__s").and_then(Value::as_u64).unwrap_or(0);
    let instance_id = obj.get("__id").and_then(Value::as_i64);
    Ok(El {
        tag: tag.to_string(),
        props,
        children,
        v,
        s,
        instance_id,
        cache: None,
    })
}

// ---- prop readers ----

pub(crate) fn prop<'a>(el: &'a El, key: &str) -> Option<&'a Value> {
    el.props.get(key).or_else(|| {
        // camelCase ↔ snake_case 双写兼容
        let snake: String = key
            .chars()
            .flat_map(|c| {
                if c.is_uppercase() {
                    vec!['_', c.to_ascii_lowercase()]
                } else {
                    vec![c]
                }
            })
            .collect();
        el.props.get(&snake)
    })
}

pub(crate) fn p_num(el: &El, key: &str) -> Result<Option<f64>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Number(n)) => Ok(n.as_f64()),
        Some(v) => Err(format!("JSX: {key} must be a number, got {v}")),
    }
}

pub(crate) fn p_vec3(el: &El, key: &str) -> Result<Option<[f64; 3]>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) if a.len() == 3 => {
            let mut out = [0.0; 3];
            for i in 0..3 {
                out[i] = a[i]
                    .as_f64()
                    .ok_or_else(|| format!("JSX: {key}[{i}] must be a number"))?;
            }
            Ok(Some(out))
        }
        Some(v) => Err(format!("JSX: {key} must be [x, y, z], got {v}")),
    }
}

pub(crate) fn p_str(el: &El, key: &str) -> Result<Option<String>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s.clone())),
        Some(v) => Err(format!("JSX: {key} must be a string, got {v}")),
    }
}

pub(crate) fn p_num_list(el: &El, key: &str) -> Result<Option<Vec<f64>>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) => a
            .iter()
            .map(|v| {
                v.as_f64()
                    .ok_or_else(|| format!("JSX: {key} items must be numbers"))
                    .map(Some)
            })
            .collect::<Result<Vec<_>, _>>()
            .map(|v| Some(v.into_iter().flatten().collect())),
        Some(v) => Err(format!("JSX: {key} must be a number list, got {v}")),
    }
}

pub(crate) fn to_arg(v: &Value) -> ArgValue {
    match v {
        Value::Number(n) => ArgValue::Num(n.as_f64().unwrap_or(0.0)),
        Value::Bool(b) => ArgValue::Bool(*b),
        Value::String(s) => ArgValue::Str(s.clone()),
        Value::Array(a) => ArgValue::List(a.iter().map(to_arg).collect()),
        _ => ArgValue::Num(0.0),
    }
}

pub(crate) const MATERIAL_KEYS: [&str; 9] = [
    "color",
    "roughness",
    "metalness",
    "emissive",
    "opacity",
    "ior",
    "absorption",
    "map",
    "unlit",
];
