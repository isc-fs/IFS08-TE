"""
Generador de Excel actualizado para mapeo CAN ID y Variables
Sistema de Telemetría ISCmetrics - ECU y AMS (5 módulos)
"""

import pandas as pd
from openpyxl import Workbook
from openpyxl.styles import PatternFill, Font, Alignment, Border, Side
from datetime import datetime

# Colores según el Excel original
COLOR_HEADER = 'FFC000'  # Naranja
COLOR_ECU = 'B7DEE8'     # Azul claro
COLOR_AMS = 'C6E0B4'     # Verde claro
COLOR_VARIABLES = 'FFF2CC'  # Amarillo claro

# Crear workbook
wb = Workbook()
ws = wb.active
ws.title = "Mapeo CAN ID"

# ============== ENCABEZADOS ==============
headers_col_a = ['Categoría', 'ID', 'Valor']
headers_col_g = ['ID', 'Nombre', 'Descripción']
headers_col_l = ['nombre variable', 'tipo de variable', 'descripción']

# Configurar encabezados
for i, header in enumerate(headers_col_a, start=1):
    cell = ws.cell(row=1, column=i)
    cell.value = header
    cell.fill = PatternFill(start_color=COLOR_HEADER, end_color=COLOR_HEADER, fill_type='solid')
    cell.font = Font(bold=True, color='000000')
    cell.alignment = Alignment(horizontal='center', vertical='center')

for i, header in enumerate(headers_col_g, start=7):
    cell = ws.cell(row=1, column=i)
    cell.value = header
    cell.fill = PatternFill(start_color=COLOR_HEADER, end_color=COLOR_HEADER, fill_type='solid')
    cell.font = Font(bold=True, color='000000')
    cell.alignment = Alignment(horizontal='center', vertical='center')

for i, header in enumerate(headers_col_l, start=12):
    cell = ws.cell(row=1, column=i)
    cell.value = header
    cell.fill = PatternFill(start_color=COLOR_HEADER, end_color=COLOR_HEADER, fill_type='solid')
    cell.font = Font(bold=True, color='000000')
    cell.alignment = Alignment(horizontal='center', vertical='center')

# ============== DATOS ECU PRINCIPAL ==============
row = 2

# Sección: IDs CAN Telemetría ECU Principal
ws.cell(row=row, column=1).value = "IDs CAN Telemetría ECU Principal"
ws.cell(row=row, column=1).fill = PatternFill(start_color=COLOR_ECU, end_color=COLOR_ECU, fill_type='solid')
ws.cell(row=row, column=1).font = Font(bold=True)

# 0x600 - DC Bus y Datos Principales
ecu_data = [
    ("", "ID_DC_BUS_BATTERY", "0x600", "0x600", "DC bus + battery info", 
     "Voltaje DC bus, potencia, RPM, torque total, voltaje mínimo celda, sensores acelerador",
     "dc_bus_voltage", "float", "Voltaje del bus DC (V)"),
    ("", "", "", "", "", "",
     "dc_bus_power", "float", "Potencia del bus DC (W)"),
    ("", "", "", "", "", "",
     "rpm", "float", "RPM del motor eléctrico"),
    ("", "", "", "", "", "",
     "torque_total", "float", "Par total del motor (Nm)"),
    ("", "", "", "", "", "",
     "cell_min_v", "float", "Voltaje mínimo de celda (mV)"),
    ("", "", "", "", "", "",
     "throttle_raw1", "float", "Lectura raw sensor acelerador 1"),
    ("", "", "", "", "", "",
     "throttle_raw2", "float", "Lectura raw sensor acelerador 2"),
    
    # 0x610 - Estado Inversor
    ("", "ID_INVERTER_STATE", "0x610", "0x610", "inverter state + RPM",
     "Temperaturas del inversor y estado operacional",
     "motor_temp", "float", "Temperatura motor (°C)"),
    ("", "", "", "", "", "",
     "pwrstg_temp", "float", "Temperatura power stage (°C)"),
    ("", "", "", "", "", "",
     "air_temp", "float", "Temperatura aire (°C)"),
    ("", "", "", "", "", "",
     "n_actual", "float", "Velocidad actual del motor"),
    ("", "", "", "", "", "",
     "i_actual", "float", "Corriente actual del inversor (A)"),
    
    # 0x620 - Pedales y Sensores
    ("", "ID_PEDALS_SENSORS", "0x620", "0x620", "pedals/sensors",
     "Lecturas raw de sensores de pedales y botones",
     "s1_raw", "float", "Sensor 1 raw"),
    ("", "", "", "", "", "",
     "s2_raw", "float", "Sensor 2 raw"),
    ("", "", "", "", "", "",
     "brake_raw", "float", "Sensor freno raw"),
    ("", "", "", "", "", "",
     "precharge_button", "float", "Estado botón precarga (0/1)"),
    ("", "", "", "", "", "",
     "start_button", "float", "Estado botón arranque (0/1)"),
    
    # 0x630 - Torque y Porcentajes
    ("", "ID_TORQUE_PEDAL_PCT", "0x630", "0x630", "torque y pedales procesados",
     "Par requerido/estimado y porcentajes de pedales",
     "torque_req", "float", "Par solicitado (Nm)"),
    ("", "", "", "", "", "",
     "torque_est", "float", "Par estimado (Nm)"),
    ("", "", "", "", "", "",
     "throttle", "float", "Acelerador procesado (0-100%)"),
    ("", "", "", "", "", "",
     "brake", "float", "Freno procesado (0-100%)"),
    
    # 0x640 - Sensores Adicionales
    ("", "ID_ADDITIONAL_SENSORS", "0x640", "0x640", "sensores adicionales",
     "Sensor de corriente y monitoreo batería",
     "current_sensor", "float", "Lectura sensor corriente"),
    ("", "", "", "", "", "",
     "cell_min_v", "float", "Voltaje mínimo celda (mV)"),
    ("", "", "", "", "", "",
     "cell_max_temp", "float", "Temperatura máxima celda (°C)"),
    
    # 0x645 - Sensores Suspensión
    ("", "ID_SUSPENSION_SENSORS", "0x645", "0x645", "sensores suspensión",
     "Array de 4 sensores de desplazamiento de suspensión + estadísticas",
     "ds_t1", "float", "Desplazamiento suspensión 1"),
    ("", "", "", "", "", "",
     "ds_t2", "float", "Desplazamiento suspensión 2"),
    ("", "", "", "", "", "",
     "ds_t3", "float", "Desplazamiento suspensión 3"),
    ("", "", "", "", "", "",
     "ds_t4", "float", "Desplazamiento suspensión 4"),
    ("", "", "", "", "", "",
     "ds_avg", "float", "Promedio de desplazamientos"),
    ("", "", "", "", "", "",
     "ds_max", "float", "Máximo desplazamiento"),
    ("", "", "", "", "", "",
     "ds_count", "float", "Contador de lecturas"),
    
    # 0x680 - Estado Sistema
    ("", "ID_SYSTEM_STATUS", "0x680", "0x680", "estado sistema",
     "Estado operacional general y banderas de error",
     "status", "float", "Estado del sistema"),
    ("", "", "", "", "", "",
     "errors", "float", "Códigos de error activos"),
]

for data_row in ecu_data:
    ws.cell(row=row, column=1).value = data_row[0]
    ws.cell(row=row, column=2).value = data_row[1]
    ws.cell(row=row, column=3).value = data_row[2]
    ws.cell(row=row, column=7).value = data_row[3]
    ws.cell(row=row, column=8).value = data_row[4]
    ws.cell(row=row, column=9).value = data_row[5]
    ws.cell(row=row, column=12).value = data_row[6]
    ws.cell(row=row, column=13).value = data_row[7]
    ws.cell(row=row, column=14).value = data_row[8]
    
    # Color de fondo para ECU
    for col in range(1, 15):
        ws.cell(row=row, column=col).fill = PatternFill(start_color=COLOR_ECU, end_color=COLOR_ECU, fill_type='solid')
    
    row += 1

# ============== DATOS AMS (5 MÓDULOS) ==============
row += 1
ws.cell(row=row, column=1).value = "IDs CAN Telemetría AMS (5 Módulos)"
ws.cell(row=row, column=1).fill = PatternFill(start_color=COLOR_AMS, end_color=COLOR_AMS, fill_type='solid')
ws.cell(row=row, column=1).font = Font(bold=True)
row += 1

ams_data = [
    # 0x201 - Corriente
    ("IDs CAN AMS", "ID_AMS_CURRENT", "0x201", "0x201", "corriente batería",
     "Corriente total del pack de baterías",
     "current_dA", "float", "Corriente en deciAmperios (dividir por 10 para obtener A)"),
    
    # 0x202 - Resumen Voltajes
    ("", "ID_AMS_VOLTAGE_SUMMARY", "0x202", "0x202", "resumen voltajes",
     "Estadísticas globales de voltaje para los 5 módulos (95 celdas totales)",
     "max_cell_mv", "float", "Voltaje máximo de celda global (mV)"),
    ("", "", "", "", "", "",
     "min_cell_mv", "float", "Voltaje mínimo de celda global (mV)"),
    ("", "", "", "", "", "",
     "stack_total_mv", "float", "Voltaje total del stack (mV)"),
    
    # 0x203-0x207 - Voltajes Módulo 0
    ("", "ID_AMS_M0_VOLT_BLK1", "0x203", "0x203", "Módulo 0 - voltajes 0-3",
     "Voltajes celdas 0-3 del módulo 0 (19 celdas totales)",
     "cell_mv[0-3]", "float[4]", "Primeras 4 celdas del módulo 0"),
    
    ("", "ID_AMS_M0_VOLT_BLK2", "0x204", "0x204", "Módulo 0 - voltajes 4-7",
     "Voltajes celdas 4-7 del módulo 0",
     "cell_mv[4-7]", "float[4]", "Celdas 4-7 del módulo 0"),
    
    ("", "ID_AMS_M0_VOLT_BLK3", "0x205", "0x205", "Módulo 0 - voltajes 8-11",
     "Voltajes celdas 8-11 del módulo 0",
     "cell_mv[8-11]", "float[4]", "Celdas 8-11 del módulo 0"),
    
    ("", "ID_AMS_M0_VOLT_BLK4", "0x206", "0x206", "Módulo 0 - voltajes 12-15",
     "Voltajes celdas 12-15 del módulo 0",
     "cell_mv[12-15]", "float[4]", "Celdas 12-15 del módulo 0"),
    
    ("", "ID_AMS_M0_VOLT_BLK5", "0x207", "0x207", "Módulo 0 - voltajes 16-18",
     "Voltajes celdas 16-18 del módulo 0 (últimas 3 celdas)",
     "cell_mv[16-18]", "float[3]", "Últimas 3 celdas del módulo 0 (19 total)"),
    
    # 0x208 - Resumen Temperaturas
    ("", "ID_AMS_TEMP_SUMMARY", "0x208", "0x208", "resumen temperaturas",
     "Estadísticas globales de temperatura para los 5 módulos (190 sensores totales)",
     "max_temp_c", "float", "Temperatura máxima global (°C)"),
    ("", "", "", "", "", "",
     "min_temp_c", "float", "Temperatura mínima global (°C)"),
    ("", "", "", "", "", "",
     "avg_temp_c", "float", "Temperatura promedio (°C, multiplicado x10)"),
    ("", "", "", "", "", "",
     "valid_count", "float", "Número de sensores válidos"),
    
    # 0x209-0x20D - Temperaturas Módulo 0
    ("", "ID_AMS_M0_TEMP_BLK1", "0x209", "0x209", "Módulo 0 - temps 0-7",
     "Sensores temperatura 0-7 del módulo 0 (38 sensores totales)",
     "temp_c[0-7]", "float[7]", "Primeros 8 sensores del módulo 0"),
    
    ("", "ID_AMS_M0_TEMP_BLK2", "0x20A", "0x20A", "Módulo 0 - temps 8-15",
     "Sensores temperatura 8-15 del módulo 0",
     "temp_c[8-15]", "float[7]", "Sensores 8-15 del módulo 0"),
    
    ("", "ID_AMS_M0_TEMP_BLK3", "0x20B", "0x20B", "Módulo 0 - temps 16-23",
     "Sensores temperatura 16-23 del módulo 0",
     "temp_c[16-23]", "float[7]", "Sensores 16-23 del módulo 0"),
    
    ("", "ID_AMS_M0_TEMP_BLK4", "0x20C", "0x20C", "Módulo 0 - temps 24-31",
     "Sensores temperatura 24-31 del módulo 0",
     "temp_c[24-31]", "float[7]", "Sensores 24-31 del módulo 0"),
    
    ("", "ID_AMS_M0_TEMP_BLK5", "0x20D", "0x20D", "Módulo 0 - temps 32-37",
     "Sensores temperatura 32-37 del módulo 0 (últimos 6 sensores)",
     "temp_c[32-37]", "float[6]", "Últimos 6 sensores del módulo 0 (38 total)"),
]

for data_row in ams_data:
    ws.cell(row=row, column=1).value = data_row[0]
    ws.cell(row=row, column=2).value = data_row[1]
    ws.cell(row=row, column=3).value = data_row[2]
    ws.cell(row=row, column=7).value = data_row[3]
    ws.cell(row=row, column=8).value = data_row[4]
    ws.cell(row=row, column=9).value = data_row[5]
    ws.cell(row=row, column=12).value = data_row[6]
    ws.cell(row=row, column=13).value = data_row[7]
    ws.cell(row=row, column=14).value = data_row[8]
    
    # Color de fondo para AMS
    for col in range(1, 15):
        ws.cell(row=row, column=col).fill = PatternFill(start_color=COLOR_AMS, end_color=COLOR_AMS, fill_type='solid')
    
    row += 1

# ============== NOTA SOBRE MÓDULOS ==============
row += 1
ws.cell(row=row, column=1).value = "NOTA: Sistema AMS de 5 Módulos"
ws.cell(row=row, column=1).font = Font(bold=True, italic=True)
row += 1
ws.cell(row=row, column=1).value = "• Cada módulo: 19 celdas + 38 sensores de temperatura"
row += 1
ws.cell(row=row, column=1).value = "• Total sistema: 95 celdas + 190 sensores de temperatura"
row += 1
ws.cell(row=row, column=1).value = "• Los IDs 0x203-0x20D se repiten para cada módulo (0-4)"
row += 1
ws.cell(row=row, column=1).value = "• El parseo en ISC_RTT_serial.py organiza datos por módulo automáticamente"

# ============== AJUSTAR ANCHOS DE COLUMNA ==============
ws.column_dimensions['A'].width = 35
ws.column_dimensions['B'].width = 25
ws.column_dimensions['C'].width = 12
ws.column_dimensions['G'].width = 12
ws.column_dimensions['H'].width = 30
ws.column_dimensions['I'].width = 70
ws.column_dimensions['L'].width = 25
ws.column_dimensions['M'].width = 20
ws.column_dimensions['N'].width = 70

# ============== GUARDAR ==============
filename = f"CANIDyVarMain_Actualizado_{datetime.now().strftime('%Y%m%d_%H%M')}.xlsx"
wb.save(filename)
print(f"✅ Excel generado exitosamente: {filename}")
print(f"📊 Contiene mapeo completo de:")
print(f"   - ECU Principal: IDs 0x600, 0x610, 0x620, 0x630, 0x640, 0x645, 0x680")
print(f"   - AMS (5 módulos): IDs 0x201-0x20D")
print(f"   - Total: 95 celdas + 190 sensores de temperatura")
