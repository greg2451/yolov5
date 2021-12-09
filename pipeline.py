import sagemaker
from sagemaker.pytorch import PyTorch
import os 
import boto3

s3_base_uri = 's3://boat-detection-dataset/'
role = 'arn:aws:iam::395354015056:role/SageMaker_role'
client = boto3.client('sagemaker')

if role is None:
    role = sagemaker.session.get_execution_role(sagemaker_session)

pytorch_estimator = PyTorch('train.py',
                            instance_type='ml.g4dn.xlarge',
                            source_dir='yolov5/',
                            instance_count=1,
                            role = role,
                            framework_version='1.8.0',
                            py_version='py3',
                            output_path='{}sagemaker-models/trained-model-output'.format(s3_base_uri),
                            metric_definitions=[
                                {'Name': 'Precision', 'Regex': 'P=(.*?);'},
                                {'Name': 'Recall', 'Regex': 'R=(.*?);'},
                                {'Name': 'mAP@0.5', 'Regex': 'map50=(.*?);'},
                                {'Name': 'mAP', 'Regex': 'map=(.*?);'}
                            ],
                            hyperparameters = {'epochs': 10, 'batch-size': 16, 'data': 'data/seaowl.yaml', 'weights' : 'exp/weights/best.pt', 'hyp' : 'data/hyps/gregdan.finetune.yaml', 'cfg' : 'models/gregdan_cfg.yaml'}, 
                            enable_sagemaker_metrics=True)

# Pour récupérer le dernier modèle en date, mettre dans les arguments 'exp/weights/best.pt': 

#Pour faire du transfer learning from scratch : 'weights' : 'yolov5x.pt'
    
# Get last model name 

listJobs = client.list_training_jobs(StatusEquals='Completed',SortBy='CreationTime')['TrainingJobSummaries']
trainingJobName = listJobs[0]['TrainingJobName']
#trainingJobName = 'pipelines-ctrtaeoo10d6-TrainModel-1CeQRdq03o'
modelUri = s3_base_uri+'sagemaker-models/trained-model-output/'+trainingJobName+'/output/'
print("Model Training ID : ", trainingJobName)
print("Location : ",modelUri) 


# Pour entrainer le modèle à la main (attention le modèle ne sera pas réutilisé automatiquement par la suite), cette fonctionalité ne sert qu'a troubleshooter
#pytorch_estimator.fit({'train': s3_base_uri+'seaowlDataset/','modelImport': modelUri})



# Pour entrainer le modèle via une pipeline : 


# Step 1 - Create Training Step
from sagemaker.inputs import TrainingInput
from sagemaker.workflow.steps import TrainingStep
from sagemaker.workflow.parameters import (
		ParameterInteger,
		ParameterString,
		ParameterFloat

	)

train_data = ParameterString(
		name="TrainDataUrl",
		default_value= s3_base_uri+'seaowlDataset/',
	)




print("Train Data : "+ s3_base_uri+'seaowlDataset/')
print("modelUri :" + modelUri)


step_train = TrainingStep(
	name="TrainModel",
	estimator=pytorch_estimator,
	inputs={
		"train": TrainingInput(
			s3_data=train_data,
		), 
        "modelImport": TrainingInput(
			s3_data=modelUri,
		),
		},
	)







# Create Pipeline
# https://github.com/aws/sagemaker-python-sdk/blob/master/src/sagemaker/workflow/pipeline.py

from sagemaker.workflow.pipeline import Pipeline
from sagemaker.workflow.execution_variables import ExecutionVariables
from sagemaker.workflow.pipeline_experiment_config import PipelineExperimentConfig


pipeline = Pipeline(
	name='yoloPipeline',
	parameters=[
        train_data,
        ],
		steps=[step_train],
	)



#Check that the pipeline is well-formed 
import json 
json.loads(pipeline.definition())


upsert_response = pipeline.upsert(role_arn=role)
print("Created/Updated SageMaker Pipeline: Response received:")
print(upsert_response)

#Start a pipeline execution
execution = pipeline.start()

print('DESCRIBE')
execution.describe()
print('LIST_STEPS')
print(execution.list_steps())
#Wait for the execution to finish

#execution.wait(delay=60,max_attempts=60)
execution.wait()

print("\n#####Execution completed. Execution step details:")