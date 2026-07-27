"""穩定 ID 指派與影像空間重鎖定。

這三個測試各自對應一個實機回報的 bug：
  1. 車一移動框就消失（舊版：追蹤器沒給 ID 就整筆丟掉）
  2. 丟失後重新偵測到卻鎖不回去
  3. 鎖不回去所以框從紅色掉回綠色
"""

from uav_yolo.vision.detector import Detection, TargetLock
from uav_yolo.vision.tracking import StableIdAssigner


def box(cx, cy, w=40.0, h=30.0):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def det(track_id, cx, cy, w=40.0, h=30.0, conf=0.9):
    return Detection(track_id=track_id, cls_name="Car", conf=conf, bbox=box(cx, cy, w, h))


# ---------------- 穩定 ID ----------------

def test_fast_moving_target_keeps_the_same_id():
    """快速移動的目標必須維持同一個 ID。

    這是「車一移動框就消失」的核心：識別要跟得上位移，且 30fps 下每幀
    移動 35px（≈ 目標一個身長）屬於很常見的速度，不該被判成新目標。
    """
    a = StableIdAssigner()
    ids = [a.assign([box(100 + 35 * i, 200)], t=i / 30.0)[0] for i in range(20)]
    assert len(set(ids)) == 1, f"移動中 ID 跳號：{ids}"


def test_id_survives_a_detection_gap():
    """偵測中斷幾幀（遮蔽/漏偵）後回來，ID 不能換人。"""
    a = StableIdAssigner()
    first = a.assign([box(100, 200)], t=0.0)[0]
    a.assign([box(120, 200)], t=1 / 30)
    for i in range(2, 12):          # 10 幀完全沒偵測到
        a.assign([], t=i / 30)
    # 依照先前速度，目標此時約在 x≈340；給一個合理的位置
    again = a.assign([box(330, 205)], t=12 / 30)[0]
    assert again == first, "偵測空窗後 ID 被換掉，鎖定就會斷"


def test_distinct_targets_get_distinct_ids():
    """兩台相隔很遠的車不能被併成同一個 ID。"""
    a = StableIdAssigner()
    ids = a.assign([box(100, 100), box(900, 700)], t=0.0)
    assert len(set(ids)) == 2
    ids2 = a.assign([box(110, 105), box(905, 695)], t=1 / 30)
    assert ids2 == ids, "位置幾乎沒變卻換了 ID"


def test_expired_track_is_dropped():
    """長期不見的軌跡要淘汰，否則舊 ID 會在很久以後劫持新目標。"""
    a = StableIdAssigner(max_missed=5)
    first = a.assign([box(100, 100)], t=0.0)[0]
    for i in range(1, 10):
        a.assign([], t=i / 30)
    again = a.assign([box(100, 100)], t=10 / 30)[0]
    assert again != first


# ---------------- 影像空間重鎖定 ----------------

def test_relock_when_id_changes_underneath():
    """鎖定的 ID 死掉、同一個目標以新 ID 出現 → 必須自動接回。

    舊版只有「世界座標重鎖」，需要 home 與 GPS 位置；室內或未取得 home 時
    那段程式碼永遠不會執行，於是重新偵測到也只能是綠框。
    """
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(7)
    assert lock.update([det(7, 300, 300)]).track_id == 7

    got = lock.update([det(42, 310, 305)])  # 同位置、換了新 ID
    assert got is not None, "同一個目標換 ID 後接不回來 → 框會變綠"
    assert got.track_id == 42
    assert lock.locked_id == 42, "鎖定要改綁到新 ID，否則之後每幀都要重找"


def test_relock_rejects_a_far_away_object():
    """重鎖不能亂抓：離最後位置太遠的物體不算同一個目標。"""
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(7)
    lock.update([det(7, 300, 300)])
    assert lock.update([det(42, 1200, 900)]) is None


def test_relock_rejects_wildly_different_size():
    """尺寸差太多（例：把整面牆當成車）不接受。"""
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(7)
    lock.update([det(7, 300, 300, w=40, h=30)])
    assert lock.update([det(42, 305, 302, w=400, h=300)]) is None


def test_relock_window_expires():
    """超過重鎖窗口就不再接回，避免很久以後抓到別台車。"""
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(7)
    lock.update([det(7, 300, 300)])
    for _ in range(TargetLock.REACQUIRE_FRAMES + 1):
        lock.update([])
    assert lock.update([det(42, 300, 300)]) is None


def test_unlock_clears_reacquire_memory():
    """使用者按解除鎖定後，不能靠殘留的影像記憶又自己鎖回去。"""
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(7)
    lock.update([det(7, 300, 300)])
    lock.unlock()
    assert lock.update([det(42, 300, 300)]) is None
    assert lock.locked_id is None
