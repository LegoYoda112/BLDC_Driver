import time
import progressbar

bar = progressbar.ProgressBar(max_value=200)
for i in range(200):
    time.sleep(0.1)
    bar.update(i)