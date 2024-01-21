from flask import Flask, request, jsonify, render_template
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import os

app = Flask(__name__)

app._static_folder = os.path.dirname(os.path.abspath(__file__))

@app.route('/')
def home():
    message = "Welcome to my ROS mini project page!"

    return render_template('index.html', message=message)

@app.route('/<robot_name>/<x>/<y>', methods=['POST'])
def post_location(robot_name, x, y):
    """Accepts POST requests and appends coordinates to a CSV file."""
    filename = f"{robot_name}_loc.csv"
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), x, y])
    return jsonify({"status": "success"})

@app.route('/<robot_name>', methods=['GET'])
def get_plot(robot_name):
    """Returns an HTML page showing a PNG plot of the robot's location."""
    filename = f"{robot_name}_loc.csv"
    x_coords, y_coords = [], []
    if os.path.exists(filename):
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                _, x, y = row
                x_coords.append(float(x))
                y_coords.append(float(y))
        plt.figure()
        plt.plot(x_coords, y_coords, marker='o')
        plt.title(f'Location Plot for {robot_name}')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plot_filename = f'{robot_name}.png'
        plt.savefig(plot_filename)
        plt.close()
        return render_template('show_plot.html', robot_name=robot_name, plot_filename=plot_filename)
    else:
        return "No data available for this robot."

@app.route('/static/<path:filename>')
def static_files(filename):
    """Serve static files (including images) from the same directory."""
    return send_from_directory(app._static_folder, filename)

@app.route('/All', methods=['GET'])
def get_all_locations():
    """Returns a JSON object containing the locations of all robots."""
    all_locations = {}
    # List all CSV files in the current directory
    csv_files = [f for f in os.listdir() if f.endswith('_loc.csv')]

    for csv_file in csv_files:
        robot_name = csv_file.replace('_loc.csv', '')
        x_coords, y_coords = [], []

        with open(csv_file, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                _, x, y = row
                x_coords.append(float(x))
                y_coords.append(float(y))

        all_locations[robot_name] = {
            'x_coords': x_coords,
            'y_coords': y_coords
        }

    return jsonify(all_locations)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

