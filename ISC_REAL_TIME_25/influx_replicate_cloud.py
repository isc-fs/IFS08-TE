
import datetime
from influxdb_client import InfluxDBClient, Point, WriteOptions  
from influxdb_client.client.write_api import SYNCHRONOUS

import requests

# InfluxDB connection details
oss_bucket = "2023-10-05 14:05:29 |> FS-range_test_2-CHUS"
oss_org = "b2b03940375e28ac" 
oss_token = "sNZJlQsv2KhY28xPDZBmt2HbgGtwU5AE4vUjQsks9WPm3o0g_UreLkxNVgMHNotIqUMHCcgAyN4llUvdXs4QtA=="
oss_url = "http://localhost:8086"

cloud_bucket = "cantoblanco_endurance_juan" 
cloud_org = "44b2810a938c40f7"
cloud_token = "Ncw6Rq19Xt1FXg3JMLS8vqToZT6RON6yRM0JmiBIAx8pZTs5RBth38iPFHu0Wfw6C71sqVEGag_IKcHOx1xyow=="
cloud_token_remote = 'DDRUOqUR7g8sMzGzYC4P8L8Ecr6Vsg6yJYLILAOq8PqZsd9MRhhe17xbE-y_s7TZx-1KO8W_xrbcCmo7ld7TgQ=='
cloud_url = "https://us-east-1-1.aws.cloud2.influxdata.com/api/v2/buckets"
cloud_url_replicate = "https://us-east-1-1.aws.cloud2.influxdata.com"

# Helper function to replicate data between time range
def replicate__():
  '''

  Depreciated
  
  '''



  # Build query with time bounds
  query = f'from(bucket:"{oss_bucket}")' \
          f'|> range(start: -30d)'
          
  
  # Create client connections
  oss_client = InfluxDBClient(url=oss_url, token=oss_token, org=oss_org)
  cloud_client = InfluxDBClient(url=cloud_url_replicate, token=cloud_token_remote, org='44b2810a938c40f7')

  # Read data from OSS
  tables = oss_client.query_api().query(query, org=oss_org)

  def success_callback(self, data):
    print(f"{data}")
    print(f"WRITE FINISHED")

  with cloud_client.write_api(success_callback=success_callback) as write_api:
     

    # Write data to Cloud
    for table in tables:
        for record in table.records:
            point = [Point(record.get_measurement()).field(record.get_field(), record.get_value())]
            # p = Point("circuito").field("data",circuito)

            #    # Write script
            # write_api = client.write_api(write_options=SYNCHRONOUS)

            # p = influxdb_client.Point("my_measurement").tag("location", "Prague").field("temperature", 25.3)
            # write_api.write(bucket=bucket, org=org, record=p)
            write_api.write(bucket='cantoblanco_endurance_juan',
                            record=point,
                            content_encoding="identity",
                            content_type="text/plain; charset=utf-8",)
            # Meter start time¿? record.get_start()
            # start = "2023-10-05T13:04:54.572000000Z" 
    write_api.close()
    print(f"Replicated data from {oss_bucket} range 30days")


def replicate(bucket):
  # Build query with time bounds
  query = f'from(bucket:"{oss_bucket}")' \
          f'|> range(start: -30d)'
          
  
  # Create client connections
  oss_client = InfluxDBClient(url=oss_url, token=oss_token, org=oss_org)
  cloud_client = InfluxDBClient(url=cloud_url_replicate, token=cloud_token_remote, org='44b2810a938c40f7')

  # Read data from OSS
  tables = oss_client.query_api().query(query, org=oss_org)


  write_api = cloud_client.write_api()
  # write_options=WriteOptions(use_timestamp=True)
  # Usando SYNCHRONOUS va de uno en uno (estamos 2 dias aqui)

  # Write data to Cloud
  for table in tables:
      for record in table.records:
          ts = int(record.values["_time"].timestamp() * 1e9)
          # Convertimos a formato timestamp
          point = [Point(record.get_measurement()).field(record.get_field(), record.get_value()).time(ts)]


          write_api.write(bucket='cantoblanco_endurance_juan',
                          record=point,
                          content_encoding="identity",
                          content_type="text/plain; charset=utf-8",)
          if record.get_measurement() == '0x600':
            print(f'Write id {record.get_measurement()} with data {record.get_field()} - {record.get_value()} at {record.values["_time"]}')

  write_api.close()
  print(f"Replicated data from {oss_bucket} range 30days")

def create_bucket():
   payloadnever = {
    "orgID": cloud_org,
    "name": 'cantoblanco_endurance_juan',
    "rp": "myrp",
    "duration":"INF",
    "schemaType": "implicit"
    }
   headers = {'Authorization': f'Token {cloud_token_remote}'}
   r1 = requests.post(cloud_url, headers=headers, json=payloadnever)
   print(r1)



def query_oss():

     # Build query with time bounds
     # 2023-09-24 18:37:53
  query = 'buckets()'

  query_2 = f'from(bucket:"{oss_bucket}")' \
          f'|> range(start: -30d)'
  
  oss_client = InfluxDBClient(url=oss_url, token=oss_token, org=oss_org)

  # Read data from OSS
  result = oss_client.query_api().query(query_2, org=oss_org)

  # Usar list comprehension mas adelante
  for table in result:
        for record in table.records:
           # print(record.values['name'])
           print(record.values['_time'])
           print(f'as timestamp {int(record.values["_time"].timestamp() * 1e9)}')
           print(record.values)

  



# create_bucket()
replicate('test')
# query_oss()