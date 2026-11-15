import pandas
import numpy as np
tibial_CS = pandas.read_excel(r"C:\Users\micha\Documents\GitHub\AAFACT4GIAS3\test_hindfoot\tibia\CoordinateSystem_tibia.xlsx", sheet_name=None)
talar_CS = pandas.read_excel(r"C:\Users\micha\Documents\GitHub\AAFACT4GIAS3\test_hindfoot\talus\CoordinateSystem_talus.xlsx", sheet_name=None)
calcaneal_CS = pandas.read_excel(r"C:\Users\micha\Documents\GitHub\AAFACT4GIAS3\test_hindfoot\calcaneus\CoordinateSystem_calcaneus.xlsx", sheet_name=None)

tibia_CS_dict = {}
for sheet_name, sheet_df in tibial_CS.items():
    sheet_name = sheet_name.split("_rbfreg_posed")[0]
    tibia_CS_dict[sheet_name] = {}    # print(sheet_df)

    tibia_CS_dict[sheet_name]["Origin"] = sheet_df.iloc[4, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    tibia_CS_dict[sheet_name]["AP"] = sheet_df.iloc[5, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    tibia_CS_dict[sheet_name]["SI"] = sheet_df.iloc[6, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    tibia_CS_dict[sheet_name]["ML"] = sheet_df.iloc[7, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)

# print(tibia_CS_dict)
talar_CS_dict = {}
for sheet_name, sheet_df in talar_CS.items():
    if "TT" not in sheet_name:
        continue
    sheet_name = sheet_name.split("TT_")[1]
    sheet_name = sheet_name.split("_rbfreg_posed")[0]
    talar_CS_dict[sheet_name] = {}

    talar_CS_dict[sheet_name]["Origin"] = sheet_df.iloc[4, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    talar_CS_dict[sheet_name]["AP"] = sheet_df.iloc[5, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    talar_CS_dict[sheet_name]["SI"] = sheet_df.iloc[6, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    talar_CS_dict[sheet_name]["ML"] = sheet_df.iloc[7, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)

calcaneal_CS_dict = {}
for sheet_name, sheet_df in calcaneal_CS.items():
    if "ST" not in sheet_name:
        continue
    sheet_name = sheet_name.split("ST_")[1]
    sheet_name = sheet_name.split("_rbfreg_posed")[0]
    calcaneal_CS_dict[sheet_name] = {}

    calcaneal_CS_dict[sheet_name]["Origin"] = sheet_df.iloc[4, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    calcaneal_CS_dict[sheet_name]["AP"] = sheet_df.iloc[5, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    calcaneal_CS_dict[sheet_name]["SI"] = sheet_df.iloc[6, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)
    calcaneal_CS_dict[sheet_name]["ML"] = sheet_df.iloc[7, 1:4].apply(pandas.to_numeric, errors="coerce").to_numpy(dtype=float)

for sheet_name in tibia_CS_dict.keys():
    ##test if the tibial SI axis (as the vector from the tibial origin to the tibial SI point) intersects the talar origin when projected along the tibial SI axis
    tibial_origin = tibia_CS_dict[sheet_name]["Origin"]
    tibial_SI = tibia_CS_dict[sheet_name]["SI"]
    talar_origin = talar_CS_dict[sheet_name]["Origin"]

    # Calculate the direction vector of the tibial SI axis
    tibial_SI_vector = tibial_SI - tibial_origin

    # Calculate the vector from the tibial origin to the talar origin
    vector_to_talar_origin = talar_origin - tibial_origin

    # Calculate the projection of the vector_to_talar_origin onto the tibial_SI_vector
    projection_length = np.dot(vector_to_talar_origin, tibial_SI_vector) / np.dot(tibial_SI_vector, tibial_SI_vector)
    projection_vector = projection_length * tibial_SI_vector

    # Calculate the intersection point
    intersection_point = tibial_origin + projection_vector

    # Check if the intersection point is close to the talar origin
    if np.allclose(intersection_point, talar_origin, atol=1e-6):
        print(f"The tibial SI axis intersects the talar origin for {sheet_name}.")
    else:
        print(f"The tibial SI axis does NOT intersect the talar origin for {sheet_name}.")
        #tell the distance between the intersection point and the talar origin
        distance = np.linalg.norm(intersection_point - talar_origin)
        print(f"The distance between the intersection point and the talar origin is {distance} mm.")

for sheet_name in calcaneal_CS_dict.keys():
    #does the calcaneal AP axis intersect the talar origin when projected along the calcaneal AP axis
    calcaneal_origin = calcaneal_CS_dict[sheet_name]["Origin"]
    calcaneal_AP = calcaneal_CS_dict[sheet_name]["AP"]
    talar_origin = talar_CS_dict[sheet_name]["Origin"]

    # Calculate the direction vector of the calcaneal AP axis
    calcaneal_AP_vector = calcaneal_AP - calcaneal_origin


    # Calculate the vector from the calcaneal origin to the talar origin
    vector_to_talar_origin = talar_origin - calcaneal_origin

    # Calculate the projection of the vector_to_talar_origin onto the calcaneal_AP_vector
    projection_length = np.dot(vector_to_talar_origin, calcaneal_AP_vector) / np.dot(calcaneal_AP_vector, calcaneal_AP_vector)
    projection_vector = projection_length * calcaneal_AP_vector

    # Calculate the intersection point
    intersection_point = calcaneal_origin + projection_vector

    # Check if the intersection point is close to the talar origin
    if np.allclose(intersection_point, talar_origin, atol=1e-6):
        print(f"The calcaneal AP axis intersects the talar origin for {sheet_name}.")
    else:
        print(f"The calcaneal AP axis does NOT intersect the talar origin for {sheet_name}.")
        #tell the distance between the intersection point and the talar origin
        distance = np.linalg.norm(intersection_point - talar_origin)
        print(f"The distance between the intersection point and the talar origin is {distance} mm.")