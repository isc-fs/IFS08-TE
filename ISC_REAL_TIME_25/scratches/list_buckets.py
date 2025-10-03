import influxdb_client
from influxdb_client.client.write_api import SYNCHRONOUS



org = "b2b03940375e28ac"
token = "sNZJlQsv2KhY28xPDZBmt2HbgGtwU5AE4vUjQsks9WPm3o0g_UreLkxNVgMHNotIqUMHCcgAyN4llUvdXs4QtA=="


# Initialize the InfluxDB client

client = influxdb_client.InfluxDBClient(url="http://localhost:8086", token=token, org=org)

# Get buckets
buckets = client.buckets.find()

# Print the list of buckets
print("List of Buckets:")
for bucket in buckets:
    print(bucket)
