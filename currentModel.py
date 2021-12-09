"""
This script will simply print the location current model that will be used during the next training on AWS 
"""
import boto3

s3_base_uri = 's3://boat-detection-dataset/'
role = 'arn:aws:iam::395354015056:role/SageMaker_role'
client = boto3.client('sagemaker')



# Get last model name 
i = 0 
listJobs = client.list_training_jobs(StatusEquals='Completed',SortBy='CreationTime')['TrainingJobSummaries']
#while not 'pipeline' in listJobs[i]['TrainingJobName']: 
#    i+=1 
    

#trainingJobName = 'pipelines-ctrtaeoo10d6-TrainModel-1CeQRdq03o'


trainingJobName = listJobs[i]['TrainingJobName']
modelUri = s3_base_uri+'sagemaker-models/trained-model-output/'+trainingJobName+'/output/'

print("Model Training ID : ", trainingJobName)
print("Location : ",modelUri) 
