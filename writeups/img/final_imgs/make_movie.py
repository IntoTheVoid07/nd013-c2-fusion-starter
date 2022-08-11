import imageio
import glob
from pathlib import Path

fp_in = "{}/movie_images/movie/*.png".format(Path().cwd())
with imageio.get_writer('/home/britt/nd013-c2-fusion-starter/sensor_fusion_project_final.gif', mode='I', fps=30) as writer:
    for filename in sorted(glob.glob(fp_in)):
        image = imageio.imread(filename)
        writer.append_data(image)
