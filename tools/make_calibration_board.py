"""產生 A4 相機校正棋盤格 PDF（橫向）。

規格與 UI 校正頁預設一致：10×7 格（= 9×6 內角點）、方格 25mm。
列印必須 100% 實際大小（不可「縮放至頁面」），頁面附 100mm 尺規供驗證。

用法：python tools/make_calibration_board.py [--compensate 0.97]
輸出：calibration_board_A4.pdf（專案根目錄）

--compensate X：印表機被迫縮放時的反向補償。例：實印 100mm 線量到 97mm
→ --compensate 0.97，整張預先放大 1/0.97，印出來就回到正確尺寸。
（註：均勻縮放其實不影響內參，補償只是讓尺寸標示誠實。）
"""

from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ap = argparse.ArgumentParser()
ap.add_argument("--compensate", type=float, default=1.0,
                help="實印縮放比（實測長度/標稱長度），例 0.97；輸出會預放大 1/比值")
args = ap.parse_args()
K = 1.0 / args.compensate  # 預放大係數

# A4 橫向（mm）
PAGE_W, PAGE_H = 297.0, 210.0
SQUARE = 25.0 * K      # 方格邊長 mm（預補償後）
COLS, ROWS = 10, 7     # 方格數（內角點 = 9×6）

BOARD_W, BOARD_H = COLS * SQUARE, ROWS * SQUARE  # 250×175
OX = (PAGE_W - BOARD_W) / 2.0                     # 置中
OY = (PAGE_H - BOARD_H) / 2.0 + 4.0               # 稍微上移，底部留說明列

MM = 1 / 25.4  # mm→inch

fig = plt.figure(figsize=(PAGE_W * MM, PAGE_H * MM))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, PAGE_W)
ax.set_ylim(0, PAGE_H)
ax.axis("off")

# 棋盤（左下角那格為黑，與 OpenCV 慣例無關緊要，黑白交錯即可）
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            ax.add_patch(Rectangle((OX + c * SQUARE, OY + r * SQUARE),
                                   SQUARE, SQUARE, facecolor="black",
                                   edgecolor="none"))

# 100mm 驗證尺規（同樣預補償；列印後量它，就該是 100mm）
ry = OY - 8.0
RULER = 100.0 * K
ax.plot([OX, OX + RULER], [ry, ry], color="black", linewidth=1.2)
for x in (OX, OX + RULER):
    ax.plot([x, x], [ry - 1.6, ry + 1.6], color="black", linewidth=1.2)
ax.text(OX + RULER / 2, ry - 3.2,
        "this line must measure exactly 100 mm  (print at 100% / actual size)",
        ha="center", va="top", fontsize=7)

# 規格標示（放頁尾，遠離棋盤不干擾角點偵測）
ax.text(PAGE_W - OX, ry - 3.2,
        "UAV_yolo camera calibration board | inner corners 9 x 6 | square 25 mm",
        ha="right", va="top", fontsize=7)

out = "calibration_board_A4.pdf"
fig.savefig(out, format="pdf")
print(f"written: {out}  (board {BOARD_W:.0f}x{BOARD_H:.0f} mm, square {SQUARE:.0f} mm, corners 9x6)")
