"""CGS 词法分析: 文本 → token 流。"""

import re


class Lexer:
    """token = (kind, value, line) 三元组。

    kind: "ident" | "number" (0x 前缀 → int 色值) | "op" (运算符)
    | 单字符符号 ("[" "]" "{" "}" "," ";" "=" "(" ")")。注释: // 到行尾。
    负号是运算符 (一元负号在表达式层处理), 不折叠进数字。
    """

    NUMBER = re.compile(r"\d+(\.\d+)?([eE][+-]?\d+)?")
    TWO_CHAR_OPS = ("==", "!=", "<=", ">=", "&&", "||")
    SINGLE_CHAR_OPS = "+-*/%<>!:"

    @staticmethod
    def tokenize(text: str) -> list[tuple[str, object, int]]:
        toks: list[tuple[str, object, int]] = []
        i, line, n = 0, 1, len(text)
        while i < n:
            ch = text[i]
            if ch == "\n":
                line += 1
                i += 1
            elif ch in " \t\r":
                i += 1
            elif text.startswith("//", i):
                while i < n and text[i] != "\n":
                    i += 1
            elif text[i : i + 2] in Lexer.TWO_CHAR_OPS:
                toks.append(("op", text[i : i + 2], line))
                i += 2
            elif ch in Lexer.SINGLE_CHAR_OPS:
                toks.append(("op", ch, line))
                i += 1
            elif ch in "[]{},;=()":
                toks.append((ch, ch, line))
                i += 1
            elif ch.isalpha() or ch == "_":
                j = i + 1
                while j < n and (text[j].isalnum() or text[j] == "_"):
                    j += 1
                toks.append(("ident", text[i:j], line))
                i = j
            elif ch.isdigit():
                if text.startswith(("0x", "0X"), i):
                    j = i + 2
                    while j < n and text[j] in "0123456789abcdefABCDEF":
                        j += 1
                    if j == i + 2:
                        raise ValueError(f"CGS 第{line}行: 非法十六进制色值")
                    toks.append(("number", int(text[i:j], 16), line))
                    i = j
                else:
                    m = Lexer.NUMBER.match(text, i)
                    if m is None:
                        raise ValueError(f"CGS 第{line}行: 无法解析的数字")
                    toks.append(("number", float(m.group(0)), line))
                    i = m.end()
            else:
                raise ValueError(f"CGS 第{line}行: 非法字符 {ch!r}")
        return toks
