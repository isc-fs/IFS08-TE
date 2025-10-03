
import csv
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

bucket = "2023-09-24 18:37:53 |> FS-Cantoblanco_Endurance_1-Juan"
org = "b2b03940375e28ac"
token = "sNZJlQsv2KhY28xPDZBmt2HbgGtwU5AE4vUjQsks9WPm3o0g_UreLkxNVgMHNotIqUMHCcgAyN4llUvdXs4QtA=="

client = InfluxDBClient(url="http://localhost:8086", token=token)



start = "2023-09-24T16:39:54.572Z"
stop = "2023-09-24T16:53:12.270Z"


exclude_measurements = ["circuito", "piloto"]
exclude_str = ','.join(exclude_measurements)
exclude_list = '[' + ','.join(exclude_str.split(',')) + ']'

query = 'from(bucket: "' + bucket + '") |> range(start: ' + start + ', stop: ' + stop +')'
filter_clause = 'fn: (r) => (r._measurement != "{}")'.format(exclude_str)
query += ' |> filter(' + filter_clause.format(exclude_list) + ')'

result = client.query_api().query(org=org, query=query)

points = []

#points = query_api.query(query)

csv_filename = f"{bucket}.csv"


for table in result:
    for record in table.records:
        point = {
            "time": record.get_time(),
            "measurement": record.get_field(), 
            "value": record.get_value()
        }
        points.append(point)

with open(csv_filename, 'a', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=["time", "measurement", "value"])
    writer.writeheader()
    for point in points:
        writer.writerow(point)