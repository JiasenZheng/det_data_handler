#!/usr/bin/python3

"""
CocoHandler class
TODO:
1. check dataset directory
2. unzip dataset if there are zip files
3. delete zip files
4. create a merged dataset folder if there is not
5. merge all dataset into one folder
6. create or update the metadata.json file
"""

import os 
import shutil
import json
import coco_assistant
import zipfile

class CocoHandler():
    def __init__(self,
                 dataset_dir):
        self.dataset_dir = dataset_dir
        self.merged_dir = os.path.join(self.dataset_dir, "merged")
        self.task_dir = os.path.join(self.dataset_dir, "task")
        assert os.path.exists(self.dataset_dir), "Dataset directory does not exist"
        assert os.path.exists(self.task_dir), "Task directory does not exist"
        if not os.path.exists(self.merged_dir):
            os.makedirs(self.merged_dir)
        self.metadata_file = os.path.join(self.merged_dir, "metadata.json")


    def unzip_tasks(self, task_dir):
        """
        Unzip all tasks
        """
        for task in os.listdir(task_dir):
            task_file = os.path.join(self.task_dir, task)
            if task_file.endswith(".zip"):
                task_dir_name = task_file.split(".zip")[0]
                new_task_dir = os.path.join(self.task_dir, task_dir_name)
                if not os.path.exists(new_task_dir):
                    os.makedirs(new_task_dir)
                # move the zip file to the new task directory
                os.rename(task_file, os.path.join(new_task_dir, task))
                # unzip the zip file
                with zipfile.ZipFile(os.path.join(new_task_dir, task), "r") as zip_ref:
                    zip_ref.extractall(new_task_dir)
                # delete the zip file
                os.remove(os.path.join(new_task_dir, task))
    
    def merge_tasks(self, task_dir):
        """
        Merge all tasks
        """
        # remove everything in the merged directory
        for file in os.listdir(self.merged_dir):
            file_path = os.path.join(self.merged_dir, file)
            if os.path.isfile(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        # create a tmp directory
        tmp_dir = os.path.join(task_dir, "tmp")
        image_dir = os.path.join(tmp_dir, "images")
        annotation_dir = os.path.join(tmp_dir, "annotations")
        if not os.path.exists(tmp_dir):
            os.makedirs(tmp_dir)
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)
        if not os.path.exists(annotation_dir):
            os.makedirs(annotation_dir)
        metadata = []
        # merge all tasks   
        for task in os.listdir(task_dir):
            if task == "tmp":
                continue
            metadata.append(task)
            task_dir = os.path.join(self.task_dir, task)
            # copy task folder to tmp/images folder
            os.system("cp -r {} {}".format(task_dir, image_dir))
            # move all images to its parent folder
            sub_task_dir = os.path.join(image_dir, task)
            sub_image_dir = os.path.join(sub_task_dir, "images")
            sub_annotation_dir = os.path.join(sub_task_dir, "annotations")
            os.system("mv {}/* {}".format(sub_image_dir, sub_task_dir))
            # remove the images folder
            os.system("rm -r {}".format(sub_image_dir))
            # rename the annotation file
            annotation_file = os.path.join(sub_annotation_dir, "instances_default.json")
            new_annotation_file = os.path.join(sub_annotation_dir, task + ".json")
            os.rename(annotation_file, new_annotation_file)
            # move the annotation file to tmp/annotations folder
            os.system("mv {} {}".format(new_annotation_file, annotation_dir))
            # remove the annotation folder
            os.system("rm -r {}".format(sub_annotation_dir))
        # merge all images
        cas = coco_assistant.COCO_Assistant(image_dir, annotation_dir)
        cas.merge()
        # move the merged images to merged folder
        result_dir = os.path.join(tmp_dir, "results", "merged")
        os.system("mv {}/* {}".format(result_dir, self.merged_dir))
        # remove the tmp folder
        os.system("rm -r {}".format(tmp_dir))
        # update the metadata file
        json.dump(metadata, open(self.metadata_file, "w"))
