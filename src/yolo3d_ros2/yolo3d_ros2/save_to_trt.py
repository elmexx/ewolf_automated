from torch_lib.Dataset import *
from torch_lib.Math import *
from torch_lib.Plotting import *
from torch_lib import Model, ClassAverages

import os
import time

import numpy as np
import cv2

import torch
import torch.nn as nn
from torch.autograd import Variable
from torchvision.models import vgg

import argparse

# os.environ['TORCH_HOME'] = '/home/jetson/Data/Data/Model'

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


parser = argparse.ArgumentParser()

parser.add_argument("--image-dir", default="eval/image_2/",
                    help="Relative path to the directory containing images to detect. Default \
                    is eval/image_2/")

# TODO: support multiple cal matrix input types
parser.add_argument("--cal-dir", default="eval/calib/",
                    help="Relative path to the directory containing camera calibration form KITTI. \
                    Default is camera_cal/")

parser.add_argument("--video", action="store_true",
                    help="Weather or not to advance frame-by-frame as fast as possible. \
                    By default, this will pull images from ./eval/video")

parser.add_argument("--show-yolo", action="store_true",
                    help="Show the 2D BoundingBox detecions on a separate image")

parser.add_argument("--hide-debug", action="store_true",
                    help="Supress the printing of each 3d location")


def plot_regressed_3d_bbox(img, cam_to_img, box_2d, dimensions, alpha, theta_ray, img_2d=None):

    # the math! returns X, the corners used for constraint
    location, X = calc_location(dimensions, cam_to_img, box_2d, alpha, theta_ray)

    orient = alpha + theta_ray

    if img_2d is not None:
        plot_2d_box(img_2d, box_2d)

    plot_3d_box(img, cam_to_img, orient, dimensions, location) # 3d boxes

    return location

if __name__ == '__main__':
    # FLAGS = parser.parse_args()

    # load 3D detection model
    weights_path = os.path.abspath(os.path.dirname(__file__)) + '/weights'
    model_lst = [x for x in sorted(os.listdir(weights_path)) if x.endswith('.pkl')]
    if len(model_lst) == 0:
        print('No previous model found, please train first!')
        exit()
    else:
        print('Using previous model %s'%model_lst[-1])
        my_vgg = vgg.vgg19_bn(pretrained=True)
        model3D = Model.Model(features=my_vgg.features, bins=2).cuda()
        checkpoint = torch.load(weights_path + '/%s'%model_lst[-1])
        model3D.load_state_dict(checkpoint['model_state_dict'])
        model3D.eval()
    
    import tensorrt as trt
    # Step 1: Convert Torch model to ONNX
    dummy_input = torch.randn([1,3,224,224]).cuda()  # Replace with your input shape
    onnx_path = "model.onnx"  # Path to save the ONNX model

    # Export Torch model to ONNX format
    torch.onnx.export(model3D, dummy_input, onnx_path, opset_version=11)

    # Step 2: Convert ONNX model to TensorRT model with FP16 precision

    trt_engine_path = "model_fp16.trt"  # Path to save the FP16 TensorRT model

    # Create a logger
    logger = trt.Logger(trt.Logger.WARNING)

    # Create a builder
    builder = trt.Builder(logger)

    # Create a build configuration specifying how TensorRT should optimize the model
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 20) # 1 MiB
    config.set_flag(trt.BuilderFlag.FP16)
    # config.set_flag(trt.BuilderFlag.INT8)
    # config.int8_calibrator = EngineCalibrator(calib_cache)
    # config.int8_calibrator.set_image_batcher(ImageBatcher)

    # Create a network definition:
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))

    # Create an ONNX parser to populate the network as follows:
    parser = trt.OnnxParser(network, logger)

    # Read the model file and process any errors:
    success = parser.parse_from_file(onnx_path)
    for idx in range(parser.num_errors):
        print(parser.get_error(idx))
    if not success:
        pass # Error handling code here

    # Engine can be built and serialized with
    serialized_engine = builder.build_serialized_network(network, config)

    # Save the engine to a file for future use
    with open(trt_engine_path, "wb") as f:
        f.write(serialized_engine)
