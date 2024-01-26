import pandas as pd
from typing import Dict

logdir_path = "/home/user/ws/src/cloud_seg_evaluation/logs/"
file_path = logdir_path + "2024-01-25-04:09:22/evaluations.csv"
num_classes = 18

data = pd.read_csv(file_path)
data["IoU"] = 0.0

total_iou: Dict[str, float] = {}
iou_count: Dict[str, int] = {}

# Iterate over each row in the DataFrame
for index, row in data.iterrows():
    # Perform some operation on each row
    # For example, print the row index and the row data
    overlap = row["positive"]
    union = row["false_positive"] + row["false_negative"] + row["positive"]
    if overlap == 0 and union == 0:
        iou = -1.0
    else:
        iou = overlap / union
        total_iou[row["label"]] = total_iou.get(row["label"], 0.0) + iou
        iou_count[row["label"]] = iou_count.get(row["label"], 0) + 1
    data.loc[index, "IoU"] = iou

sum_total_iou = 0.0
sum_iou_count = 0
for key in total_iou.keys():
    sum_total_iou += total_iou[key]
    sum_iou_count += iou_count[key]
    print(f'{key}: {total_iou[key] / iou_count[key]}')
print(f'mIoU: {sum_total_iou / sum_iou_count}')
