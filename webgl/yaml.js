// 最小 YAML 解析器 —— 覆盖 CRDF 使用的子集:
//   块映射 (key: value), 同缩进块序列 (key:\n- item 或 顶层 - item),
//   序列项可吸收后续深缩进键, 流式序列 [..]/流式映射 {..}, 注释 (#),
//   标量 (数字/字符串/布尔/null)。

export function parseYAML(text) {
  const lines = text.split(/\r?\n/).map((raw) => {
    const line = raw.replace(/#.*$/, "").replace(/\s+$/, "");
    return { indent: line.match(/^\s*/)[0].length, content: line.trim() };
  });
  let pos = 0;

  function peek() {
    return lines[pos];
  }
  function isSeqItem(c) {
    return c.startsWith("-") && (c.length === 1 || c[1] === " " || c[1] === "\t");
  }

  function parseValue(s) {
    if (s.startsWith("[")) return parseFlow(s, 0).value;
    if (s.startsWith("{")) return parseFlow(s, 0).value;
    if (s === "" || s === "null" || s === "~") return null;
    if (s === "true") return true;
    if (s === "false") return false;
    if (/^-?\d+$/.test(s)) return parseInt(s, 10);
    if (/^-?\d*\.\d+([eE][+-]?\d+)?$/.test(s) || /^[+-]?\d+[eE][+-]?\d+$/.test(s)) {
      return parseFloat(s);
    }
    if (s.startsWith('"') && s.endsWith('"') && s.length >= 2) return s.slice(1, -1);
    if (s.startsWith("'") && s.endsWith("'") && s.length >= 2) return s.slice(1, -1);
    return s;
  }

  function parseFlow(s, start) {
    if (s[start] === "[") {
      const arr = [];
      let i = start + 1;
      while (i < s.length && s[i] !== "]") {
        const r = parseFlow(s, i);
        arr.push(r.value);
        i = r.end;
        while (i < s.length && (s[i] === "," || s[i] === " ")) i++;
      }
      return { value: arr, end: i + 1 };
    }
    if (s[start] === "{") {
      const obj = {};
      let i = start + 1;
      while (i < s.length && s[i] !== "}") {
        const k = parseFlow(s, i);
        i = k.end;
        while (i < s.length && (s[i] === " " || s[i] === ":")) i++;
        const v = parseFlow(s, i);
        obj[k.value] = v.value;
        i = v.end;
        while (i < s.length && (s[i] === "," || s[i] === " ")) i++;
      }
      return { value: obj, end: i + 1 };
    }
    let i = start;
    while (i < s.length && !",]}\"'".includes(s[i]) && s[i] !== " ") i++;
    return { value: parseValue(s.slice(start, i)), end: i };
  }

  // 序列: 项在固定缩进; 每项可吸收后续更深缩进的键
  function parseSequence(indent) {
    const arr = [];
    while (pos < lines.length) {
      const { indent: i2, content: c2 } = peek();
      if (i2 !== indent || !isSeqItem(c2)) break;
      const rest = c2.slice(1).trim();
      pos++;
      if (rest === "") {
        arr.push(parseBlock(indent + 2));
      } else if (rest.includes(":")) {
        const key = rest.slice(0, rest.indexOf(":")).trim();
        const val = rest.slice(rest.indexOf(":") + 1).trim();
        const item = {};
        if (val === "") {
          item[key] = peekIndent(indent) ? parseBlock(indent + 2) : null;
        } else {
          item[key] = parseValue(val);
        }
        if (peek() && peek().indent > indent) {
          Object.assign(item, parseBlock(indent + 2));
        }
        arr.push(item);
      } else {
        arr.push(parseValue(rest));
        if (peek() && peek().indent > indent) arr.push(parseBlock(indent + 2));
      }
    }
    return arr;
  }

  function peekIndent(parent) {
    return peek() && (peek().indent > parent || (peek().indent === parent && isSeqItem(peek().content)));
  }

  function parseBlock(minIndent) {
    const node = {};
    while (pos < lines.length) {
      const { indent, content } = peek();
      if (indent < minIndent) break;
      if (content === "") {
        pos++;
        continue;
      }
      if (isSeqItem(content) && indent === minIndent) {
        return parseSequence(indent); // 整个块是序列 (如顶层)
      }
      const ci = content.indexOf(":");
      if (ci < 0) throw new Error("YAML 解析失败: 期望 key: " + content);
      const key = content.slice(0, ci).trim();
      const rest = content.slice(ci + 1).trim();
      pos++;
      if (rest === "") {
        if (peek() && isSeqItem(peek().content) && peek().indent >= indent) {
          node[key] = parseSequence(peek().indent); // 同缩进序列
        } else if (peek() && peek().indent > indent) {
          node[key] = parseBlock(indent + 2);
        } else {
          node[key] = null;
        }
      } else {
        node[key] = parseValue(rest);
      }
    }
    return node;
  }

  return parseBlock(0);
}
