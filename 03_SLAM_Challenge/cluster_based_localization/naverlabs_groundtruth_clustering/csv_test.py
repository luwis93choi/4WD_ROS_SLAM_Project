import csv

fieldnames = ['1st_name', '2nd_name']

with open('./test.csv', 'w', encoding='utf-8') as file: 

    writer = csv.DictWriter(file, fieldnames=fieldnames)

    writer.writeheader()

    writer.writerow({'1st_name' : 'Baked', '2nd_name' : 'Beans'})
