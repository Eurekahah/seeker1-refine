import cv2, numpy as np, time

def rss_kb():
    for line in open('/proc/self/status'):
        if line.startswith('VmRSS:'):
            return int(line.split()[1])
    return -1

w, h = 960, 480
imgs = [np.random.randint(0, 256, (1200, 1920, 3), np.uint8) for _ in range(5)]
mx = np.random.uniform(-1, 1921, (h, w)).astype(np.float32)
my = np.random.uniform(-1, 1201, (h, w)).astype(np.float32)
mx16, my16 = cv2.convertMaps(mx, my, cv2.CV_16SC2)
print('start', rss_kb(), flush=True)
for name, x, y in [('f32', mx, my), ('i16', mx16, my16)]:
    out = np.empty((h, w, 3), np.uint8)
    t0 = time.time()
    for i in range(160):
        cv2.remap(imgs[i % len(imgs)], x, y, cv2.INTER_LINEAR, dst=out, borderMode=cv2.BORDER_CONSTANT)
        if i % 20 == 0:
            print(name, i, rss_kb(), flush=True)
    print(name, 'end', rss_kb(), 'fps', round(160/(time.time()-t0),1), flush=True)
