from influxdb_client import InfluxDBClient, Point  

cloud_org = "44b2810a938c40f7"
cloud_token = "Ncw6Rq19Xt1FXg3JMLS8vqToZT6RON6yRM0JmiBIAx8pZTs5RBth38iPFHu0Wfw6C71sqVEGag_IKcHOx1xyow=="
cloud_url = "https://us-east-1-1.aws.cloud2.influxdata.com"


query = 'buckets()'

cloud_client = InfluxDBClient(url=cloud_url, token=cloud_token, org=cloud_org)

tables = cloud_client.query_api().query(query, org=cloud_org)
print(tables)
for table in tables:
    for record in table.records:
        print(record)