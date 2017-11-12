from __future__ import print_function
from os import path, listdir, makedirs
from random import shuffle
import argparse
from shutil import copy2

def split_dataset(
    new_data_base,old_dataset_base,
    val_split=.1,test_split=.1,
    allowed_image_extentions={".jpg",".jpeg",".png"},
    ordering = shuffle):
    
    assert(0<= val_split and val_split <= 1)
    assert(0<= test_split and test_split <= 1)
    assert(val_split + test_split <= 1)

    train_base = path.join(new_data_base,"train")
    test_base = path.join(new_data_base,"test")
    validation_base = path.join(new_data_base,"validation")
    
    classes = [name for name in listdir(old_dataset_base) if path.isdir(path.join(old_dataset_base,name)) and name[0] != "."]
    for class_label in classes:
        
        image_files = [name for name in listdir(path.join(old_dataset_base,class_label)) if path.splitext(name)[1] in allowed_image_extentions]
        ordering(image_files)
        n_images = len(image_files)
        n_val_files = int(n_images * val_split)
        n_test_files = int(n_images * test_split)

        val_files = image_files[0:n_val_files]
        test_files = image_files[n_val_files:n_val_files+n_test_files]
        train_files = image_files[n_val_files+n_test_files:]
        
        if len(val_files) > 0:
            makedirs(path.join(validation_base,class_label))
        
        if len(test_files) > 0:
            makedirs(path.join(test_base,class_label))
        
        if len(train_files) > 0:
            makedirs(path.join(train_base,class_label))
        
        for val_file in val_files:
            copy2( path.join(old_dataset_base,class_label,val_file),
                    path.join(validation_base,class_label,val_file))

        for test_file in test_files:
            copy2( path.join(old_dataset_base,class_label,test_file),
                    path.join(test_base,class_label,test_file))

        for train_file in train_files:
            copy2( path.join(old_dataset_base,class_label,train_file),
                    path.join(train_base,class_label,train_file))
    


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Splits a dataset into train, test, and validation parts by copying")
    parser.add_argument("--source",help="The source dataset base folder",default=".",type=str)
    parser.add_argument("--dest",help="Location of the new base folder",default=".",type=str)
    parser.add_argument("--valsplit",help="Fraction of dataset to put in validation folder",default=0.1,type=float)
    parser.add_argument("--testsplit",help="Fraction of dataset to put in test folder",default=0.1,type=float)
    args = parser.parse_args()
    split_dataset(args.dest,args.source,args.valsplit,args.testsplit)