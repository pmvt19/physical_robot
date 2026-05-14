# 1. Create the output directory (if it doesn't exist)
mkdir -p generated

# 2. Run the protoc command
python -m grpc_tools.protoc -I. --python_out=generated/ --pyi_out=generated/ --grpc_python_out=generated/ robot_data.proto

# 3. Warn user to change the import in the robot_data_pb2_grpc.py file to import generated.robot_data_pb2
echo "IMPORTANT: Change import in robot_data_pb2_grpc.py from import robot_data_pb2 to import generated.robot_data_pb2"