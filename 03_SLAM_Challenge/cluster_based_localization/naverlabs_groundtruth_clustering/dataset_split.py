import csv
import os

from sklearn.model_selection import train_test_split

class dataset_splitter():

    @staticmethod
    def split(full_set_path, test_ratio=0.3):

        print('[INFO] Splitting dataset / Train ratio : ' + str(1-test_ratio) + ' / Test ratio : ' + str(test_ratio))

        for datasetPath in full_set_path:

            full_set = []
            train_set = []
            valid_set = []
            with open(datasetPath, 'r', encoding='utf-8') as datasetfile:

                reader = csv.DictReader(datasetfile)

                for dataDict in reader:

                    full_set.append(dataDict)

                train_set, valid_set = train_test_split(full_set, test_size=test_ratio, shuffle=True)

            base = os.path.basename(datasetPath)
            base = os.path.splitext(base)[0]

            fieldnames = ['ImgName', 'Cluster_Label', 'Coordinate', 'Quaternion']
            with open('./' + base + '_train.csv', 'w', encoding='utf-8') as datasetfile:

                writer = csv.DictWriter(datasetfile, fieldnames=fieldnames)

                writer.writeheader()

                for i in range(len(train_set)):

                    writer.writerow(train_set[i])

            with open('./' + base + '_valid.csv', 'w', encoding='utf-8') as datasetfile:

                writer = csv.DictWriter(datasetfile, fieldnames=fieldnames)

                writer.writeheader()

                for i in range(len(valid_set)):

                    writer.writerow(valid_set[i])