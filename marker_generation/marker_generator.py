import math
import cv2
import numpy as np
from alive_progress import alive_bar
import click
from termcolor import colored

def bytesToBits(bytes_list, length=0):
    '''
    Convert bytes into bits
    '''
    bits = ''
    for a in bytes_list:
        b = bin(a)[2:]
        b = '0' * (8 - len(b)) + b
        bits = bits + b
    # Add 0 in the end
    if length != 0:
        bits = bits + '0' * (length - len(bits))

    return bits


def bitsToBytes(bits_list):
    '''
    Convert bits into bytes
    '''
    assert (len(bits_list) % 8 == 0)
    bytes_list = []
    cnt = 0
    while cnt < len(bits_list):
        binary = '0b' + bits_list[cnt:cnt + 8]
        bytes_list.append(int(binary, 2))
        cnt += 8
    return bytearray(bytes_list)

'''
Exception class for terminating the stream
'''
class TerminateStream(Exception):
    pass


class MarkerGenerator:
    def __init__(self, input_path, fps, cells_payload):
        self.fps = fps
        self.cells_payload = cells_payload
        self.cells_inner_locator = 1
        # TODO: Make enough border cells for larger marker
        # self.cells_border = int(math.ceil(cells_payload / 16))
        self.cells_border = 1
        self.cells_outer_locator = 3 * self.cells_border
        # TODO: Make the cell pixel size adjustable
        self.pixels_cell = int(640 / cells_payload)
        self.pixels_inner_locator = self.cells_inner_locator * self.pixels_cell
        self.pixels_outer_locator = self.cells_outer_locator * self.pixels_cell
        self.pixels_pattern = (cells_payload + 2 * (
                    self.cells_inner_locator + self.cells_outer_locator)) * self.pixels_cell
        self.pixels_payload = self.cells_payload * self.pixels_cell
        self.bits_per_frame = cells_payload ** 2
        self.bytes = bytearray()
        with open(input_path, 'rb') as f:
            self.bytes = f.read()
        self.bits = bytesToBits(self.bytes)
        self.bits = self.bits + '0' * (self.bits_per_frame - len(self.bits) % self.bits_per_frame)
        self.num_frames = int(len(self.bits) / self.bits_per_frame)

    def encode(self, bits):
        data = np.zeros((self.cells_payload, self.cells_payload))
        for i in range(len(bits)):
            x = i % self.cells_payload
            y = i // self.cells_payload
            data[y, x] = int(bits[i])
        return data

    @staticmethod
    def addInnerLocator(data):
        data = np.pad(data, 1, constant_values=1)
        for i in range(data.shape[0]):
            if i % 2 == 1:
                data[0, i] = 0
                data[-1 - i, -1] = 0
        data[0, -1] = 1
        return data

    def addOuterLocator(self, data):
        data = np.pad(data, self.cells_border, constant_values=0)
        data = np.pad(data, self.cells_border, constant_values=1)
        data = np.pad(data, self.cells_border, constant_values=0)
        return data

    def makeCells(self, data):
        rows = data.shape[0]
        cols = data.shape[1]
        c = self.pixels_cell
        image = np.zeros((rows * c, cols * c, 3))
        for j in range(rows):
            for i in range(cols):
                y = j * c
                x = i * c
                if data[j, i] == 1:
                    image[y:y + c, x:x + c, :] = 255 * np.ones((c, c, 3))
        image = image.astype(np.uint8)
        return image

    def generate(self, video_path, duration):
        writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps,
                                 (self.pixels_pattern, self.pixels_pattern))
        cnt = 0
        white = np.ones((self.cells_payload, self.cells_payload))
        black = np.zeros((self.cells_payload, self.cells_payload))
        white_frame = self.makeCells(self.addOuterLocator(self.addInnerLocator(white)))
        black_frame = self.makeCells(self.addOuterLocator(self.addInnerLocator(black)))
        black_blank_frame = self.makeCells(self.addOuterLocator(np.pad(black, 1, constant_values=0)))
        try:
            with alive_bar(self.fps * duration) as bar:
                for i in range(int(self.fps * 3)):
                    writer.write(black_blank_frame)
                    bar()
                    cnt += 1
                # Add blinking markers at the beginning
                for i in range(int(self.fps * 3 / 2)):
                    writer.write(white_frame)
                    bar()
                    writer.write(black_blank_frame)
                    bar()
                    cnt += 2
                payload = np.zeros((self.cells_payload, self.cells_payload))
                for i in range(self.num_frames):
                    bit_idx = i * self.bits_per_frame
                    bits = self.bits[bit_idx:bit_idx + self.bits_per_frame]
                    if cnt >= self.fps * duration:
                        raise TerminateStream()
                    payload = self.encode(bits)
                    data = self.addInnerLocator(payload)
                    data = self.addOuterLocator(data)
                    frame = self.makeCells(data)
                    writer.write(frame)
                    bar()
                    # Insert a pure black image after each data frame
                    writer.write(black_blank_frame)
                    bar()
                    cnt += 2
            writer.release()
        except TerminateStream:
            writer.release()


@click.command()
@click.argument("data_path", type=click.Path(exists=True))
@click.option("--output_path", type=click.Path(), default="marker.mp4", help="Path to save the marker sequence in mp4")
@click.option("--cell", type=int, default=16, help="Cell number of marker (in the multiple of 8, e.g. 16, 32, 64, ...)")
@click.option("--fps", type=int, default=60, help="Frame-rate of marker (in Hz)")
@click.option("--duration", type=int, default=30, help="The duration of marker (in seconds)")
def generate_marker(data_path, output_path, cell, fps, duration):
    print("Reading data from {}".format(data_path))
    print("Set marker fps to {} Hz, duration to {} seconds".format(fps, duration))
    if cell % 8 == 0:
        print("Generating markers with cell number {}".format(cell))
    else:
        print(colored("[WARNING]: Cell number is not a multiple of 8.", "yellow"))

    generator = MarkerGenerator(data_path, fps, cell)
    try:
        generator.generate(output_path, duration)
        print("Marker generated successfully!")
    except:
        print("Error in generating marker") 


if __name__ == '__main__':
    generate_marker()