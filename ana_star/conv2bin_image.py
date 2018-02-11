import sys
from PIL import Image


if __name__ == "__main__":
    assert len(sys.argv) == 2, "Incorrect Number of arguments" # require image input
    im_name = str(sys.argv[1])

    im = Image.open(im_name)
    l = im.convert('1')
    l.save('b_'+im_name)