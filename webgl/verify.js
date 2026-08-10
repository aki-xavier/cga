// WebGL 渲染器验证: console 错误 + 像素读回 + 关节滑块联动
const { chromium } = require("/Users/aki/code/cv/node_modules/playwright");

(async () => {
  const browser = await chromium.launch({
    executablePath:
      "/Users/aki/Library/Caches/ms-playwright/chromium-1134/chrome-mac/Chromium.app/Contents/MacOS/Chromium",
    headless: true,
    args: [],
  });
  const page = await browser.newPage({ viewport: { width: 1000, height: 700 } });
  const errors = [];
  page.on("console", (m) => { if (m.type() === "error") errors.push(m.text()); });
  page.on("pageerror", (e) => errors.push(String(e)));
  await page.goto("http://localhost:8123/index.html", { waitUntil: "load" });
  await page.waitForTimeout(800);

  const snap = () => page.evaluate(() => window.__snapshot(48));
  const s0 = await snap();
  const bg = [135, 206, 235];
  let arm = 0;
  for (let i = 0; i < s0.length; i += 4) {
    const r = s0[i], g = s0[i + 1], b = s0[i + 2];
    if (Math.abs(r - bg[0]) > 15 || Math.abs(g - bg[1]) > 15 || Math.abs(b - bg[2]) > 15) arm++;
  }
  console.log("console errors:", errors.length ? errors : "无");
  console.log(`非背景像素: ${arm}/2304 (${(100 * arm / 2304).toFixed(1)}%)`);

  // 关节滑块联动: 改 joint2 → 像素应变化
  await page.evaluate(() => {
    const r = document.querySelector("#c").__renderer || null;
    // 通过滑块 input 事件触发
    const inputs = document.querySelectorAll("#joints input");
    inputs[1].value = "1.0";
    inputs[1].dispatchEvent(new Event("input"));
  });
  await page.waitForTimeout(300);
  const s1 = await snap();
  let diff = 0;
  for (let i = 0; i < s0.length; i += 4) {
    if (Math.abs(s0[i] - s1[i]) + Math.abs(s0[i + 1] - s1[i + 1]) + Math.abs(s0[i + 2] - s1[i + 2]) > 12) diff++;
  }
  console.log(`joint2=1.0 后变化像素: ${diff}/2304 (${(100 * diff / 2304).toFixed(1)}%)`);
  console.log(diff > 50 ? "滑块联动 ✓" : "滑块联动 ✗ (无变化)");

  await browser.close();
  process.exit(errors.length ? 1 : 0);
})().catch((e) => { console.error("FAIL:", e.message); process.exit(1); });
