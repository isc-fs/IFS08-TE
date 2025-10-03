token = 'sNZJlQsv2KhY28xPDZBmt2HbgGtwU5AE4vUjQsks9WPm3o0g_UreLkxNVgMHNotIqUMHCcgAyN4llUvdXs4QtA=='
# All access token
org = 'b2b03940375e28ac'
# Org id
bucket = "296f7f6218c651a2"
# Bucket id for ISC tmp buckettoken = 'sNZJlQsv2KhY28xPDZBmt2HbgGtwU5AE4vUjQsks9WPm3o0g_UreLkxNVgMHNotIqUMHCcgAyN4llUvdXs4QtA=='

def writerundata(bucketnever,bucket,piloto,circuito):
    write_api = client.write_api(write_options=SYNCHRONOUS)


    p = Point("piloto").field("data",piloto)
    write_api.write(bucket=bucketnever, record=p)
    write_api.write(bucket=bucket, record=p)


    p = Point("circuito").field("data",circuito)
    write_api.write(bucket=bucketnever, record=p)
    write_api.write(bucket=bucket, record=p)


def rundata():


    piloto = input("Introducir el nombre del piloto: ")
    circuito = input("Introducir el nombre del circuito: ")

    return piloto,circuito



def createbucket(piloto,circuito):
    headers = {'Authorization': f'Token {token}'}
    url = "http://localhost:8086/api/v2/buckets"
    payloadtemp = {
    "orgID": org,
    "name": "ISC",
    "description": "create a bucket",
    "rp": "myrp",
    "retentionRules":[
    {
    "type": "expire",
    "everySeconds": 86400
    }
    ]
    }

    test = datetime.now()
    time = test.strftime("%Y-%m-%d %H:%M:%S")
    print(time)

    payloadnever = {
    "orgID": org,
    "name": time+" |> FS-"+circuito+"-"+piloto,
    "description": "create a bucket",
    "rp": "myrp",
    "duration":"INF"
    }

    # "type": "expire","everySeconds": 86400 // es 1 dia
    #Para crear un rp que no se borre quitas retentionRules y pones "duration": "INF"
    #No se puede crear un bucket con el mismo nombre que uno ya existente

    r1 = requests.post(url, headers=headers, json=payloadnever)
    r2 = requests.post(url, headers=headers, json=payloadtemp)
    #print(r.text.split(",")[0].split(":")[1].replace('"',"").replace(" ",""))
    #print(r.text)
    return r1.text.split(",")[0].split(":")[1].replace('"',"").replace(" ","")
    #r.text.split(",")[0].split(":")[1].replace('"',"").replace(" ","") de aqui sacas el id del bucket creado
