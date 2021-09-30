# -*- encoding:utf-8 -*-
"""VISAPP2020 Dataset Classes

Original
https://github.com/amdegroot/ssd.pytorch
voc0712.py
"""
from .config import HOME
import os.path as osp
import sys
import torch
from torch.autograd import Variable
import torch.utils.data as data
import cv2
import numpy as np
import json
import glob

WRS2020_CLASSES = (  # always index 0
    "01-BASE",
    "03-PLATE2",
    "02-PLATE",
    "04_37D-GEARMOTOR-50-70",
    "11_MBRAC60-2-10",
    "07_SBARB6200ZZ_30",
    "13_MBGA30-2",
    "13_MBGA30-2_r",
    "05_MBRFA30-2-P6",
    "14_BGPSL6-9-L30-F7",
    "08_SSFHRT10-75-M4-FC55-G20",
    "06_MBT4-400",
    "09_EDCS10",
    "09_EDCS10_r",
    "12_CLBUS6-9-9.5",
    "10_CLBPS10_17_4",
    "03-PLATE2_r",
    "02-PLATE_r",
    "11_MBRAC60-2-10_r",
    "07_SBARB6200ZZ_30",
    "12_CLBUS6-9-9.5_r",
)
WRS_ROOT = osp.join(HOME, "WRS2020/wrs_dataset")


class AnnotationTransform(object):
    """Transforms an annotation into a Tensor of bbox coords and label index
    Initilized with a dictionary lookup of classnames to indexes

    Arguments:
        class_to_ind (dict, optional): dictionary lookup of classnames -> indexes
            (default: alphabetic indexing of VOC's 20 classes)
        keep_difficult (bool, optional): keep difficult instances or not
            (default: False)
        height (int): height
        width (int): width
    """

    def __init__(self, class_to_ind=None, keep_difficult=False):

        # self.class_to_ind = class_to_ind or dict(
        #    zip(VISAPP2020_CLASSES, range(len(VISAPP2020_CLASSES))) )
        self.keep_difficult = keep_difficult

    def __call__(self, target, width, height):
        """
        Arguments:
            target (annotation) : json file
        Returns:
            a list containing lists of bounding boxes  [bbox coords, class name]
        """
        res = []

        bboxes = target["bbox"]
        class_id = target["class_id"]
        for bbox, cid in zip(bboxes, class_id):
            bndbox = []
            # クラス名をIDに変換
            label_idx = int(cid)
            # BBoxを取り出して画像サイズで正規化
            bndbox.append(float(bbox[0]) / width)
            bndbox.append(float(bbox[1]) / height)
            bndbox.append(float(bbox[2]) / width)
            bndbox.append(float(bbox[3]) / height)
            bndbox.append(label_idx)
            res += [bndbox]  # [xmin, ymin, xmax, ymax, label_ind]

        return res  # [[xmin, ymin, xmax, ymax, label_ind], ... ]


class WRS2020_Detection(data.Dataset):
    """VISAPP2020 Detection Dataset Object

    input is image, target is annotation

    Arguments:
        root (string): filepath to VISAPP2020(Dataset root) folder.
        image_set (string): imageset to use (eg. 'train', 'val', 'test')
        transform (callable, optional): transformation to perform on the
            input image
        target_transform (callable, optional): transformation to perform on the
            target `annotation`
            (eg: take in caption string, return tensor of word indices)
        dataset_name (string, optional): which dataset to load
            (default: 'VISAPP2020')
    """

    def __init__(
        self,
        root,
        image_set=["train", "test"],
        transform=None,
        target_transform=AnnotationTransform(),
        dataset_name="wrs_dataset",
    ):
        self.root = root  # wrs_dataset/
        self.image_set = image_set

        self.transform = transform
        self.target_transform = target_transform
        self.name = dataset_name
        self._annopath = osp.join("Annotations", "Far", "Image-wise")
        self._imgpath = osp.join("Images", "Far", "RGB")
        self.ids = list()

        filelist_anno = glob.glob(osp.join(self.root, self._annopath, "*"))

        with open(osp.join(root, self._annopath, self.image_set + ".txt")) as f:
            for line in f:
                anno_name = line.strip()
                anno_fullpath = osp.join(root, self._annopath, anno_name)
                img_name = anno_name[:-5]
                img_fullpath = osp.join(root, self._imgpath, img_name)
                self.ids.append((img_fullpath, anno_fullpath))
            f.close()

    def __getitem__(self, index):
        im, gt, h, w = self.pull_item(index)

        return im, gt

    def __len__(self):
        return len(self.ids)

    def pull_item(self, index):

        im_name = self.ids[index][0]
        json_name = self.ids[index][1]

        if osp.isfile(json_name) == False:
            print(json_name, "was not be found.")
        if osp.isfile(im_name) == False:
            print(im_name, "was not be found.")

        anno = open(json_name)
        target = json.load(anno)
        img = cv2.imread(im_name)
        height, width, channels = img.shape

        if self.target_transform is not None:
            target = self.target_transform(target, width, height)

        if self.transform is not None:
            target = np.array(target)
            img, boxes, labels = self.transform(img, target[:, :4], target[:, 4])
            # to rgb
            img = img[:, :, (2, 1, 0)]
            # img = img.transpose(2, 0, 1)
            target = np.hstack((boxes, np.expand_dims(labels, axis=1)))
        return torch.from_numpy(img).permute(2, 0, 1), target, height, width
        # return torch.from_numpy(img), target, height, width

    def pull_image(self, index):
        """Returns the original image object at index in PIL form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            numpy array
        """
        return cv2.imread(self.ids[index][0], cv2.IMREAD_COLOR)

    def pull_anno(self, index):
        """Returns the original annotation of image at index

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to get annotation of
        Return:
            list:  [img_path, [(label, bbox coords),...]]
                eg: ('../wrs_dataset/Images/Far/RGB/rgb0.png', [('dog', (96, 13, 438, 332))])
        """
        anno = open(self.ids[index][1])
        target = json.load(anno)
        gt = self.target_transform(target, 1, 1)
        return self.ids[index][0], gt

    def pull_tensor(self, index):
        """Returns the original image at an index in tensor form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            tensorized version of img, squeezed
        """
        return torch.Tensor(self.pull_image(index)).unsqueeze_(0)

    def pull_path(self, index):

        return self.ids[index][0], self.ids[index][1]


# initの段階ですべての画像を読み込むバージョン．ファイルアクセス回数削減


class WRS2020_Detection2(data.Dataset):
    """VISAPP2020 Detection Dataset Object

    input is image, target is annotation

    Arguments:
        root (string): filepath to VISAPP2020(Dataset root) folder.
        image_set (string): imageset to use (eg. 'train', 'val', 'test')
        transform (callable, optional): transformation to perform on the
            input image
        target_transform (callable, optional): transformation to perform on the
            target `annotation`
            (eg: take in caption string, return tensor of word indices)
        dataset_name (string, optional): which dataset to load
            (default: 'VISAPP2020')
    """

    def __init__(
        self,
        root,
        image_set=["train", "test"],
        transform=None,
        target_transform=AnnotationTransform(),
        dataset_name="wrs_dataset",
    ):
        self.root = root  # wrs_dataset/
        self.image_set = image_set

        self.transform = transform
        self.target_transform = target_transform
        self.name = dataset_name
        self._annopath = osp.join("Annotations", "Far", "Image-wise")
        self._imgpath = osp.join("Images", "Far", "RGB")
        self.ids = list()

        filelist_anno = glob.glob(osp.join(self.root, self._annopath, "*"))

        self.imgs = list()
        self.targets = list()
        with open(osp.join(root, self._annopath, self.image_set + ".txt")) as f:
            for line in f:
                anno_name = line.strip()
                anno_fullpath = osp.join(root, self._annopath, anno_name)
                img_name = anno_name[:-5]
                img_fullpath = osp.join(root, self._imgpath, img_name)
                self.ids.append((img_fullpath, anno_fullpath))

                if osp.isfile(anno_fullpath) == False:
                    print(anno_fullpath, "was not be found.")
                if osp.isfile(img_fullpath) == False:
                    print(img_fullpath, "was not be found.")

                anno = open(anno_fullpath)
                target = json.load(anno)
                img = cv2.imread(img_fullpath)
                self.imgs.append(img)
                self.targets.append(target)

            f.close()

    def __getitem__(self, index):
        im, gt, h, w = self.pull_item(index)

        return im, gt

    def __len__(self):
        return len(self.ids)

    def pull_item(self, index):

        target = self.targets[index]
        img = self.imgs[index].copy()
        height, width, channels = img.shape

        if self.target_transform is not None:
            target = self.target_transform(target, width, height)

        if self.transform is not None:
            target = np.array(target)
            img, boxes, labels = self.transform(img, target[:, :4], target[:, 4])
            # to rgb
            img = img[:, :, (2, 1, 0)]
            # img = img.transpose(2, 0, 1)
            target = np.hstack((boxes, np.expand_dims(labels, axis=1)))
        return torch.from_numpy(img).permute(2, 0, 1), target, height, width
        # return torch.from_numpy(img), target, height, width

    def pull_image(self, index):
        """Returns the original image object at index in PIL form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            numpy array
        """
        return self.imgs[index]

    def pull_anno(self, index):
        """Returns the original annotation of image at index

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to get annotation of
        Return:
            list:  [img_path, [(label, bbox coords),...]]
                eg: ('../wrs_dataset/Images/Far/RGB/rgb0.png', [('dog', (96, 13, 438, 332))])
        """
        target = self.targets[index]
        gt = self.target_transform(target, 1, 1)
        return self.ids[index][0], gt

    def pull_tensor(self, index):
        """Returns the original image at an index in tensor form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            tensorized version of img, squeezed
        """
        return torch.Tensor(self.pull_image(index)).unsqueeze_(0)

    def pull_path(self, index):

        return self.ids[index][0], self.ids[index][1]
