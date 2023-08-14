# Copyright (c) Facebook, Inc. and its affiliates.
# Modified by Bowen Cheng from: https://github.com/facebookresearch/detectron2/blob/master/demo/demo.py
import argparse
import glob
import multiprocessing as mp
import os

# fmt: off

# fmt: on

import tempfile
import time
import warnings

import cv2
import numpy as np
import tqdm

from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.projects.deeplab import add_deeplab_config
from detectron2.utils.logger import setup_logger

from SeMask_Segmentation.SeMask_Mask2Former.mask2former import add_maskformer2_config
from SeMask_Segmentation.SeMask_Mask2Former.demo.predictor import VisualizationDemo


# constants
WINDOW_NAME = "mask2former demo"


def setup_cfg(args):
    # load config from file and command-line arguments
    cfg = get_cfg()
    add_deeplab_config(cfg)
    add_maskformer2_config(cfg)
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    cfg.freeze()
    return cfg


def get_parser():
    parser = argparse.ArgumentParser(description="maskformer2 demo for builtin configs")
    parser.add_argument(
        "--config-file",
        default="./configs/ade20k/maskformer2_semask_swin_large_IN21k_384_bs16_160k_res640.yaml",
        metavar="FILE",
        help="path to config file",
    )
    parser.add_argument(
        "--input",
        nargs="+",
        help="A list of space separated input images; "
        "or a single glob pattern such as 'directory/*.jpg'",
    )
    parser.add_argument(
        "--output",
        help="A file or directory to save output visualizations. "
        "If not given, will show output in an OpenCV window.",
    )

    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.5,
        help="Minimum score for instance predictions to be shown",
    )
    parser.add_argument(
        "--opts",
        help="Modify config options using the command-line 'KEY VALUE' pairs",
        default=['MODEL.WEIGHTS', './semask_large_mask2former_ade20k.pth'],
        nargs=argparse.REMAINDER,
    )
    return parser

if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    args = get_parser().parse_args()
    setup_logger(name="fvcore")
    logger = setup_logger()
    logger.info("Arguments: " + str(args))

    cfg = setup_cfg(args)
    demo = VisualizationDemo(cfg)

    # Syn Dataset
    input_root = "/mnt/nas_8/datasets/tiancr/IRS/Home"
    output_root = "/mnt/nas_8/group/weihong/PolarMVS/SynLabel2/Home"
    subsets = ["ArchVizInterior03Data"]

    # if args.input:
    for subset in subsets:
        subset_root = os.path.join(input_root, subset)
        submask_root = os.path.join(output_root, subset)
        subvis_root = os.path.join(output_root, subset)
        subtxt_root = os.path.join(output_root, subset)
        assert subset_root, "The input path({:s}) was not found".format(subset_root)
        os.makedirs(subvis_root, exist_ok=True)
        os.makedirs(subtxt_root, exist_ok=True)
        os.makedirs(submask_root, exist_ok=True)
        print(submask_root)

        if os.path.isdir(subset_root):
            input = []
            for path in os.listdir(subset_root):
                if path.startswith('l_'):
                    input.append(os.path.join(subset_root, path))
            args.input = sorted(input)
        labels = []
        for path in tqdm.tqdm(args.input):
            # use PIL, to be consistent with evaluation
            img = read_image(path, format="BGR")
            index = int(os.path.split(path)[1][2:-4])
            predictions, visualized_output = demo.run_on_image(img)

            sem_seg = predictions["sem_seg"].argmax(dim=0).to("cpu").numpy()
            mask_path = os.path.join(submask_root, 'semseg_'+str(index)+'.png')
            cv2.imwrite(mask_path, sem_seg)
            txt_path = os.path.join(subtxt_root, 'semseg_'+str(index)+'.txt')
            np.set_printoptions(threshold=np.inf)
            np.savetxt(txt_path, sem_seg, fmt='%i')

            # ''' save visualized output'''
            vis_path = os.path.join(subvis_root, 'seg_'+str(index)+'.png')
            visualized_output.save(vis_path)