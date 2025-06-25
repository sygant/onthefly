import argparse
import importlib
import os

import cv2
import numpy as np
import torch
import torch.distributed
import yaml
from PIL import Image, ImageOps
from scipy.special import softmax
from torch.backends import cudnn

from doppelgangers.third_party.loftr import LoFTR, default_cfg
from doppelgangers.utils.database import image_ids_to_pair_id, pair_id_to_image_ids


def dict2namespace(config):
    namespace = argparse.Namespace()
    for key, value in config.items():
        if isinstance(value, dict):
            new_value = dict2namespace(value)
        else:
            new_value = value
        setattr(namespace, key, new_value)
    return namespace

def get_args():
    class Args:
        config = r"D:\deep-image-matching\doppelgangers\configs\test_configs\disambiguation_example.yaml"
        colmap_exe_command = 'colmap'
        matching_type = 'exhaustive_matcher'  # exhaustive_matcher,vocab_tree_matcher
        skip_feature_matching = False
        database_path = None
        skip_reconstruction = True
        input_image_path = 'D:/deep-image-matching/test-image'
        output_path = None
        threshold = 0.8
        gpu = None
        pretrained = 'D:/deep-image-matching/doppelgangers/weights/doppelgangers_classifier_loftr.pt'

    args = Args()

    def dict2namespace(config):
        namespace = argparse.Namespace()
        for key, value in config.items():
            if isinstance(value, dict):
                new_value = dict2namespace(value)
            else:
                new_value = value
            setattr(namespace, key, new_value)
        return namespace

    # parse config file
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)
    config = dict2namespace(config)

    return args, config


def read_image(img_pth, img_size, df, padding):
    if str(img_pth).endswith('gif'):

        pil_image = ImageOps.grayscale(Image.open(str(img_pth)))
        img_raw = np.array(pil_image)
    else:
        img_raw = cv2.imread(img_pth, cv2.IMREAD_GRAYSCALE)

    w, h = img_raw.shape[1], img_raw.shape[0]
    w_new, h_new = get_resized_wh(w, h, img_size)
    w_new, h_new = get_divisible_wh(w_new, h_new, df)

    if padding:  # padding
        pad_to = max(h_new, w_new)
        mask = np.zeros((1, pad_to, pad_to), dtype=bool)
        mask[:, :h_new, :w_new] = True
        mask = mask[:, ::8, ::8]

    image = cv2.resize(img_raw, (w_new, h_new))
    pad_image = np.zeros((1, 1, pad_to, pad_to), dtype=np.float32)
    pad_image[0, 0, :h_new, :w_new] = image / 255.

    return pad_image, mask


def get_resized_wh(w, h, resize=None):
    if resize is not None:  # resize the longer edge
        scale = resize / max(h, w)
        w_new, h_new = int(round(w * scale)), int(round(h * scale))
    else:
        w_new, h_new = w, h
    return w_new, h_new


def get_divisible_wh(w, h, df=None):
    if df is not None:
        w_new, h_new = map(lambda x: int(x // df * df), [w, h])
    else:
        w_new, h_new = w, h
    if w_new == 0:
        w_new = df
    if h_new == 0:
        h_new = df
    return w_new, h_new


def doppelgangers_classifier(gpu, ngpus_per_node, cfg, args):
    # basic setup
    cudnn.benchmark = True
    multi_gpu = False
    strict = True

    # initial dataset
    data_lib = importlib.import_module(cfg.data.type)
    loaders = data_lib.get_data_loaders(cfg.data)
    test_loader = loaders['test_loader']

    # initial model
    decoder_lib = importlib.import_module(cfg.models.decoder.type)
    decoder = decoder_lib.decoder(cfg.models.decoder)
    decoder = decoder.cuda()

    # load pretrained model
    ckpt = torch.load(args.pretrained)
    import copy
    new_ckpt = copy.deepcopy(ckpt['dec'])
    if not multi_gpu:
        for key, value in ckpt['dec'].items():
            if 'module.' in key:
                new_ckpt[key[len('module.'):]] = new_ckpt.pop(key)
    elif multi_gpu:
        for key, value in ckpt['dec'].items():
            if 'module.' not in key:
                new_ckpt['module.' + key] = new_ckpt.pop(key)
    decoder.load_state_dict(new_ckpt, strict=strict)

    # evaluate on test set
    decoder.eval()
    gt_list = list()
    pred_list = list()
    prob_list = list()
    with torch.no_grad():
        for bidx, data in enumerate(test_loader):
            data['image'] = data['image'].cuda()
            gt = data['gt'].cuda()
            score = decoder(data['image'])
            for i in range(score.shape[0]):
                prob_list.append(score[i].cpu().numpy())
                pred_list.append(torch.argmax(score, dim=1)[i].cpu().numpy())
                gt_list.append(gt[i].cpu().numpy())

    gt_list = np.array(gt_list).reshape(-1)
    pred_list = np.array(pred_list).reshape(-1)
    prob_list = np.array(prob_list).reshape(-1, 2)
    np.save(os.path.join(cfg.data.output_path, "pair_probability_list.npy"),
            {'pred': pred_list, 'gt': gt_list, 'prob': prob_list})
    return prob_list



def GetSimilar(image0:str, image1:str,pairid, threshold=0.9):
    image0 = image0.replace("\\", "/")
    image1 = image1.replace("\\", "/")
    image0 = image0.split('/')[-1]
    image1 = image1.split('/')[-1]
    with open(r"D:\GWT\On_the_fly_SfME\src\Feature\GlobalFeature\doppelgangers\configs\test_configs\disambiguation_example.yaml", 'r') as f:
        config = yaml.safe_load(f)
    cfg = dict2namespace(config)
    result = np.load(os.path.join(cfg.data.output_path, "pair_probability_list.npy"),
                     allow_pickle=True).item()
    y_scores = np.array(result['prob']).reshape(-1, 2)
    y_scores = softmax(y_scores, axis=1)[:, 1]

    pairs_info = np.load(cfg.data.test.pair_path)
    pairs_id = np.array(pairs_info)[:, -1]
    # print(f"pairs info:\n{pairs_info}\npairs_id:\n{pairs_id}")
    # print(f"{image0}{image1}")
    print('number of matches in database: ', len(y_scores))
    print(y_scores)
    SimilarList = []
    for i in range(len(pairs_info)):
        if y_scores[i] < threshold:
            SimilarList.append(f"{pairs_info[i][0]}{pairs_info[i][1]}")
            if image0 in f"{pairs_info[i][0]}{pairs_info[i][1]}" and image1 in f"{pairs_info[i][0]}{pairs_info[i][1]}":
                return False
    # print(SimilarList)
    # print(f"{image0}{image1}")

    return True


def SimilarStructure(image0, image1, pairs_id, matches, output_path):
    # image0,image1 is the image path
    # matches is the matches between img0 and img1
    # extracting loftr matches

    args, cfg = get_args()
    # args.input_image_path = image0
    gpu = args.gpu
    ngpus_per_node = torch.cuda.device_count()

    # edit config file with corresponding data path
    cfg.data.image_dir = args.input_image_path
    cfg.data.loftr_match_dir = output_path + "/loftr_match"
    cfg.data.test.pair_path = '%s/pairs_list.npy' % output_path
    cfg.data.output_path = output_path
    threshold = 0.8

    print("Extracting loftr matches")
    loftr_matches_path = os.path.join(output_path, 'loftr_match')
    os.makedirs(loftr_matches_path, exist_ok=True)

    # 组成影像对
    pairs_list = []
    label = 0
    name0 = image0.split('/')[-1]
    name1 = image1.split('/')[-1]
    pairs_list.append([name0, name1, label, matches.shape[0], pairs_id])
    pairs_list = np.concatenate(pairs_list, axis=0).reshape(-1, 5)
    np.save('%s/pairs_list.npy' % output_path, pairs_list)

    # Loftr匹配
    matcher = LoFTR(config=default_cfg)
    model_weight_path = "D:/deep-image-matching/doppelgangers/weights/outdoor_ds.ckpt"
    matcher.load_state_dict(torch.load(model_weight_path)['state_dict'])
    matcher = matcher.eval().cuda()

    img_size = 1024
    df = 8
    padding = True

    img0_raw, mask0 = read_image(image0, img_size, df, padding)
    img1_raw, mask1 = read_image(image1, img_size, df, padding)
    img0 = torch.from_numpy(img0_raw).cuda()
    img1 = torch.from_numpy(img1_raw).cuda()
    mask0 = torch.from_numpy(mask0).cuda()
    mask1 = torch.from_numpy(mask1).cuda()
    batch = {'image0': img0, 'image1': img1, 'mask0': mask0, 'mask1': mask1}

    # Inference with LoFTR and get prediction
    with torch.no_grad():
        matcher(batch)
        mkpts0 = batch['mkpts0_f'].cpu().numpy()
        mkpts1 = batch['mkpts1_f'].cpu().numpy()
        mconf = batch['mconf'].cpu().numpy()

    output_dir = os.path.join(output_path, f'loftr_match/0.npy')
    np.save(output_dir, {"kpt0": mkpts0, "kpt1": mkpts1, "conf": mconf})

    # 运行相似纹理判断
    print("Running Doppelgangers classifier model on image pairs")
    prob = doppelgangers_classifier(gpu, ngpus_per_node, cfg, args)

    y_scores = np.array(prob).reshape(-1, 2)
    y_scores = softmax(y_scores, axis=1)[:, 1]

    if y_scores < threshold:
        SimStructure = False
    if y_scores >= threshold:
        SimStructure = True

    return SimStructure


if __name__ == '__main__':
    pipeline = "superpoint+lightglue"
    img0 = "D:/deep-image-matching/test-image/0011.jpg"
    img1 = "D:/deep-image-matching/test-image/0012.jpg"

    output_path = "D:/deep-image-matching/out"
    # sim_structure = SimilarStructure(img0, img1, 100, matches, output_path)
    GetSimilar(0, 1)

    print("Finish")
