import csv
import os

from sklearn.model_selection import train_test_split

class dataset_splitter():

    @staticmethod
    def split(full_set_path, test_ratio=0.3):

        save_files = []
        
        for datasetPath in full_set_path:

            print('[INFO] Splitting dataset : ' + datasetPath + ' / Train ratio : ' + str(1-test_ratio) + ' / Test ratio : ' + str(test_ratio))

            full_set = []
            train_set = []
            valid_set = []

            save_format = {'train' : '', 'valid' : ''}

            with open(datasetPath, 'r', encoding='utf-8') as datasetfile:

                reader = csv.DictReader(datasetfile)

                for dataDict in reader:

                    full_set.append(dataDict)

                train_set, valid_set = train_test_split(full_set, test_size=test_ratio, shuffle=True)

            base = os.path.basename(datasetPath)
            base = os.path.splitext(base)[0]

            fieldnames = ['ImgName', 'Cluster_Label', 'Coordinate', 'Quaternion']
            with open('./train_' + base + '.csv', 'w', encoding='utf-8') as datasetfile:

                writer = csv.DictWriter(datasetfile, fieldnames=fieldnames)

                writer.writeheader()

                for i in range(len(train_set)):

                    writer.writerow(train_set[i])

                save_format['train'] = './train_' + base + '.csv'

            with open('./valid_' + base + '.csv', 'w', encoding='utf-8') as datasetfile:

                writer = csv.DictWriter(datasetfile, fieldnames=fieldnames)

                writer.writeheader()

                for i in range(len(valid_set)):

                    writer.writerow(valid_set[i])

                save_format['valid'] = './valid_' + base + '.csv'

            save_files.append(save_format)

        return save_files