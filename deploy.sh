#!/bin/bash 



name=`aws sagemaker list-training-jobs --max-items=1 --sort-by='CreationTime' --status-equals='Completed' |grep TrainingJobName | sed 's|"|\n|g' |grep pipeline`
aws s3 cp 's3://boat-detection-dataset/sagemaker-models/trained-model-output/'$name'/output/model.tar.gz' model.tar.gz

tar -xvzf model.tar.gz
cp exp/weights/best.pt yolov5 
rm -r exp/


cd yolov5/runs/detect/
rm -r exp2
cd ../..
python3 detect.py --weights=best.pt --conf-thres 0.1 
shotwell /home/guest/yolov5/runs/detect/exp2/bus.jpg
