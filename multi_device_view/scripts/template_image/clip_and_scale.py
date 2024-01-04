import cv2
import sys
import argparse
import os


def crop_and_save_image(image_path, top_left_x, top_left_y, width, height):

    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Unable to read the image at {image_path}")
        sys.exit(1)

    crop_region = image[top_left_y:top_left_y + height, top_left_x:top_left_x + width]

    base_name, ext = os.path.splitext(image_path)
    output_path = base_name + "_cliped.png"
    cv2.imwrite(output_path, crop_region)

    print(f"Image cropped and saved to {output_path}")

if __name__ == "__main__":
    
    if len(sys.argv) != 6:
        print("Usage: python crop_image.py <image_path> <output_path> <top_left_x> <top_left_y> <width> <height>")
        sys.exit(1)

    # parser = argparse.ArgumentParser()

    # parser.add_argument("path")
    # parsre.add_argument(")
     
    image_path = sys.argv[1]
    top_left_x = int(sys.argv[2])
    top_left_y = int(sys.argv[3])
    width = int(sys.argv[4])
    height = int(sys.argv[5])

    # output_path = sys.argv[2]

    # 切り抜きと保存の関数を呼び出す
    crop_and_save_image(image_path, top_left_x, top_left_y, width, height)
