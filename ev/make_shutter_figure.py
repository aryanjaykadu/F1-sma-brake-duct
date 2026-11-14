"""Side-by-side render of the SMA grille-shutter module, closed (cold coolant) and open (warm coolant)."""
import os

from PIL import Image, ImageChops, ImageDraw, ImageFont

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cad")


def crop(path, pad=30):
    im = Image.open(path).convert("RGB")
    bg = Image.new("RGB", im.size, im.getpixel((5, 5)))
    diff = ImageChops.difference(im, bg).convert("L").point(lambda v: 255 if v > 40 else 0)
    x0, y0, x1, y1 = diff.getbbox()
    return im.crop((max(0, x0 - pad), max(0, y0 - pad), min(im.width, x1 + pad), min(im.height, y1 + pad)))


ims = [crop(os.path.join(D, f"EV_Shutter_{t}_trimetric.png")) for t in ("Closed", "Open")]
h = max(i.height for i in ims)
out = Image.new("RGB", (sum(i.width for i in ims) + 40, h + 70), "white")
font = ImageFont.truetype("arial.ttf", 26)
x = 0
for im, label in zip(ims, ("Coolant below 35 °C: louvers shut", "Coolant above 40 °C: louvers open 80°")):
    out.paste(im, (x, 70))
    ImageDraw.Draw(out).text((x + 20, 20), label, fill="black", font=font)
    x += im.width + 40
out.save(os.path.join(os.path.dirname(D), "EV_Shutter_Closed_vs_Open.png"))
print(out.size)
