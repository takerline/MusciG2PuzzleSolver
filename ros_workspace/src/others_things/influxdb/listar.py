from influxdb_client import InfluxDBClient

# Configuración
# Configuración
url = "https://deusto-influxdb-001-v2qrmk5znqme3f.eu-west-1.timestream-influxdb.amazonaws.com:8086"  # URL del servidor InfluxDB
token = "KgD5Y20nFrV4pKgAm_YGTr01wZJsrheSoxtM5pQFrsWNWzynEJCJiIYY0Fq1U9oeQSu1E0s9GiIGQw0tjXLxzg=="  # Token de autenticación
org = "deusto"  # Organización (configurada en la interfaz web)
bucket = "Grupo_6"  # Bucket donde se insertarán los datos

# Crear cliente
client = InfluxDBClient(url=url, token=token, org=org)

# Consulta de ejemplo
query = f"""
from(bucket: "{bucket}")
  |> range(start: -100y)  // Ajusta el rango si es necesario
"""

# Ejecutar consulta
query_api = client.query_api()
result = query_api.query(query)

# Procesar y mostrar resultados
for table in result:
    for record in table.records:
        print(f"Time: {record.get_time()}, Measurement: {record.get_measurement()}, Field: {record.get_field()}, Value: {record.get_value()}")
