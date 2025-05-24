import eventlet
eventlet.monkey_patch() 
import os
from flask import Flask, render_template, request, jsonify, redirect, url_for, flash, send_file
from flask_sqlalchemy import SQLAlchemy
from flask_login import LoginManager, UserMixin, login_user, logout_user, login_required, current_user
from werkzeug.security import generate_password_hash, check_password_hash
from flask_wtf import FlaskForm
from wtforms import StringField, PasswordField, SubmitField
from wtforms.validators import DataRequired, Email, EqualTo, ValidationError
import googlemaps
from geopy.distance import geodesic
from datetime import datetime, timedelta
import math
import pytz
import json
from math import radians, sin, cos, sqrt, atan2
from sqlalchemy import text
import folium
from flask_socketio import SocketIO
from flask_cors import CORS
import logging
logging.basicConfig(level=logging.DEBUG)


# Flask Application Configuration
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins="*")

app.config['SECRET_KEY'] = 'rummy'
app.config['SQLALCHEMY_DATABASE_URI'] ='postgresql://postgres:Allah%40786@localhost/bus'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['GOOGLE_MAPS_API_KEY'] = 'AIzaSyAvJK1GzfKt7ksiZxxshHDL2tnefB1ifJI'

# Initialize Extensions
db = SQLAlchemy(app)
login_manager = LoginManager(app)
login_manager.login_view = 'login'
gmaps = googlemaps.Client(key=app.config['GOOGLE_MAPS_API_KEY'])

# Database Models
class User(UserMixin, db.Model):
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(50), unique=True, nullable=False)
    email = db.Column(db.String(120), unique=True, nullable=False)
    password_hash = db.Column(db.String(255), nullable=False)
    is_admin = db.Column(db.Boolean, default=False)

    def set_password(self, password):
        self.password_hash = generate_password_hash(password)

    def check_password(self, password):
        return check_password_hash(self.password_hash, password)
class Feedback(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    user_id = db.Column(db.Integer, nullable=False)  # Not a foreign key
    feedback_type = db.Column(db.String(50), nullable=False)
    description = db.Column(db.Text, nullable=False)
    rating = db.Column(db.Integer, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    is_resolved = db.Column(db.Boolean, default=False)
class Stop(db.Model):
    __tablename__ = 'stops'

    stop_id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    description = db.Column(db.Text)  # Added description field
    is_active = db.Column(db.Boolean, default=True)  # Added is_active field

    def __repr__(self):
        return f"<Stop {self.name} ({self.latitude}, {self.longitude})>"

class Vehicle(db.Model):
    __tablename__ = 'vehicles'
    vehicle_id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    registration_number = db.Column(db.String(50))

# class LiveTracking(db.Model):
#     __tablename__ = 'live_tracking'
#     tracking_id = db.Column(db.Integer, primary_key=True)
#     vehicle_id = db.Column(db.Integer, db.ForeignKey('vehicles.vehicle_id'))
#     current_latitude = db.Column(db.Float, nullable=False)
#     current_longitude = db.Column(db.Float, nullable=False)
#     timestamp = db.Column(db.DateTime, default=datetime.utcnow)
#     speed = db.Column(db.Float)
#     vehicle = db.relationship('Vehicle', backref='tracking')
class Schedule(db.Model):
    __tablename__ = 'schedule'
    
    schedule_id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    route_id = db.Column(db.Integer, db.ForeignKey('routes.route_id'), nullable=False)
    vehicle_id = db.Column(db.Integer, db.ForeignKey('vehicles.vehicle_id'), nullable=False)
    start_time = db.Column(db.Time, nullable=False)
    day_of_week = db.Column(db.String(20), nullable=False)
    is_active = db.Column(db.Boolean, default=True)

    # Relationships
    route = db.relationship('Route', backref=db.backref('schedules', lazy=True))
    vehicle = db.relationship('Vehicle', backref=db.backref('schedules', lazy=True))

    def __repr__(self):
        return f"<Schedule {self.schedule_id} - Route {self.route_id} at {self.start_time}>"

class LiveTracking(db.Model):
    __tablename__ = 'live_tracking'
    tracking_id = db.Column(db.Integer, primary_key=True)
    vehicle_id = db.Column(db.Integer, db.ForeignKey('vehicles.vehicle_id'))
    route_id = db.Column(db.Integer, db.ForeignKey('routes.route_id'), nullable=True)
    current_latitude = db.Column(db.Float, nullable=False)
    current_longitude = db.Column(db.Float, nullable=False)
    current_stop_id = db.Column(db.Integer, db.ForeignKey('stops.stop_id'), nullable=True)
    next_stop_id = db.Column(db.Integer, db.ForeignKey('stops.stop_id'), nullable=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    speed = db.Column(db.Float)
    heading = db.Column(db.Float, nullable=True)
    is_delayed = db.Column(db.Boolean, default=False)
    delay_minutes = db.Column(db.Integer, default=0)
    vehicle = db.relationship('Vehicle', backref='tracking')

class Route(db.Model):
    __tablename__ = 'routes'
    
    route_id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    name = db.Column(db.String(100), nullable=False)
    description = db.Column(db.Text, nullable=True)
    color_code = db.Column(db.String(20), nullable=True)
    is_active = db.Column(db.Boolean, default=True)
    
    def __repr__(self):
        return f"<Route {self.name}>"
    
class RouteStop(db.Model):
    __tablename__ = 'route_stops'
    
    route_stop_id = db.Column(db.Integer, primary_key=True)  # Changed from 'id' to match DB schema
    route_id = db.Column(db.Integer, db.ForeignKey('routes.route_id'), nullable=False)
    stop_id = db.Column(db.Integer, db.ForeignKey('stops.stop_id'), nullable=False)
    stop_sequence = db.Column(db.Integer, nullable=False)
    distance_from_previous = db.Column(db.Float, nullable=True)
    time_from_previous = db.Column(db.Integer, nullable=True)  # Added this field from your schema
    
    # Relationships
    route = db.relationship('Route', backref=db.backref('route_stops', lazy=True))
    stop = db.relationship('Stop', backref=db.backref('route_stops', lazy=True))
    
    def __init__(self, route_id, stop_id, stop_sequence, distance_from_previous=None, time_from_previous=None):
        self.route_id = route_id
        self.stop_id = stop_id
        self.stop_sequence = stop_sequence
        self.distance_from_previous = distance_from_previous
        self.time_from_previous = time_from_previous
    
    def __repr__(self):
        return f'<RouteStop {self.route_id}-{self.stop_id} (Seq: {self.stop_sequence})>'
class LoginForm(FlaskForm):
    email = StringField('Email', validators=[DataRequired(), Email()])
    password = PasswordField('Password', validators=[DataRequired()])
    submit = SubmitField('Login')

class RegistrationForm(FlaskForm):
    username = StringField('Username', validators=[DataRequired()])
    email = StringField('Email', validators=[DataRequired(), Email()])
    password = PasswordField('Password', validators=[DataRequired()])
    confirm_password = PasswordField('Confirm Password', 
        validators=[DataRequired(), EqualTo('password')])
    submit = SubmitField('Register')

    def validate_username(self, username):
        user = User.query.filter_by(username=username.data).first()
        if user:
            raise ValidationError('Username already exists')

    def validate_email(self, email):
        user = User.query.filter_by(email=email.data).first()
        if user:
            raise ValidationError('Email already registered')

# User Loader for Flask-Login
@login_manager.user_loader
def load_user(user_id):
    return User.query.get(int(user_id))


# Authentication Routes
@app.route('/login', methods=['GET', 'POST'])
def login():
    form = LoginForm()
    if form.validate_on_submit():
        user = User.query.filter_by(email=form.email.data).first()
        if user and user.check_password(form.password.data):
            login_user(user)
            return redirect(url_for('dashboard'))
        flash('Invalid email or password')
    return render_template('login.html', form=form)

@app.route('/register', methods=['GET', 'POST'])
def register():
    form = RegistrationForm()
    if form.validate_on_submit():
        user = User(username=form.username.data, email=form.email.data)
        user.set_password(form.password.data)
        db.session.add(user)
        db.session.commit()
        flash('Registration successful!')
        return redirect(url_for('login'))
    return render_template('register.html', form=form)

@app.route('/logout')
@login_required
def logout():
    logout_user()
    return redirect(url_for('login'))

@app.route('/')
def home():
    return render_template('home.html')
# Bus Tracking Routes
@app.route('/dashboard', methods=['GET', 'POST'])
@login_required
def dashboard():
    # Handle GET request (default dashboard view)
    if request.method == 'GET':
        return render_template('dashboard.html', current_user=current_user)
    
    # Handle POST request
    elif request.method == 'POST':
        # Check if it's a specific dashboard-related action
        action = request.form.get('action') or request.json.get('action')
        
        if action == 'refresh_data':
            # Example of refreshing dashboard data
            try:
                # Fetch latest tracking information
                latest_vehicles = LiveTracking.query.order_by(LiveTracking.timestamp.desc()).limit(5).all()
                
                # Prepare vehicle data for response
                vehicle_data = [{
                    'vehicle_id': tracking.vehicle_id,
                    'latitude': tracking.current_latitude,
                    'longitude': tracking.current_longitude,
                    'timestamp': tracking.timestamp.isoformat(),
                    'speed': tracking.speed
                } for tracking in latest_vehicles]
                
                return jsonify({
                    'status': 'success',
                    'vehicles': vehicle_data
                }), 200
            
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
        
        elif action == 'get_bus_stats':
            # Example of getting bus statistics
            try:
                total_vehicles = Vehicle.query.count()
                active_vehicles = LiveTracking.query.filter(LiveTracking.timestamp >= datetime.utcnow() - timedelta(hours=1)).count()
                
                return jsonify({
                    'status': 'success',
                    'total_vehicles': total_vehicles,
                    'active_vehicles': active_vehicles
                }), 200
            
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
        
        # Default response for unrecognized POST request
        return jsonify({
            'status': 'error',
            'message': 'Invalid action'
        }), 400
# here mu

# Add this function directly in your app.py file, before your route definition
def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees) using Haversine formula.
    
    Returns distance in kilometers.
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    # Radius of earth in kilometers
    radius = 6371.0
    
    # Calculate the distance
    distance = radius * c
    
    return distance

@app.route('/show_nearest_stop', methods=['POST'])
@login_required
def show_nearest_stop():
    # Get user's current location
    user_lat = float(request.form.get('latitude'))
    user_lon = float(request.form.get('longitude'))
    
    # Find nearest bus stop
    stops = Stop.query.filter_by(is_active=True).all()
    nearest_stop = None
    min_distance = float('inf')
    
    for stop in stops:
        distance = haversine_distance(
            user_lat, user_lon,
            stop.latitude, stop.longitude
        )
        
        if distance < min_distance:
            min_distance = distance
            nearest_stop = stop
    
    if not nearest_stop:
        flash("No active bus stops found.", "error")
        return redirect(url_for('dashboard'))
    
    # Find routes that pass through this stop
    route_stops = RouteStop.query.filter_by(stop_id=nearest_stop.stop_id).all()
    route_ids = [rs.route_id for rs in route_stops]
    routes = Route.query.filter(Route.route_id.in_(route_ids), Route.is_active==True).all()
    
    # Get available routes using the helper function
    available_routes = get_available_routes(nearest_stop.stop_id)
    
    # Check if there's an eta_info from a previous calculation
    eta_info = request.args.get('eta_info')
    if eta_info:
        eta_info = json.loads(eta_info)
    
    return render_template('nearest_stop.html',
        stop_id=nearest_stop.stop_id,
        user_lat=user_lat,
        user_lon=user_lon,
        stop_lat=nearest_stop.latitude,
        stop_lon=nearest_stop.longitude,
        distance=min_distance,
        stop_name=nearest_stop.name,
        available_routes=available_routes,
        eta_info=eta_info,
        google_maps_api_key=app.config['GOOGLE_MAPS_API_KEY']
    )

@app.route('/calculate_eta', methods=['POST'])
@login_required
def calculate_eta():
    # Get form data
    current_stop_id = int(request.form.get('current_stop_id'))
    user_lat = float(request.form.get('user_lat'))
    user_lon = float(request.form.get('user_lon'))
    route_id = int(request.form.get('route_id'))
    
    # Get the current stop and route information
    current_stop = Stop.query.get(current_stop_id)
    selected_route = Route.query.get(route_id)
    
    if not current_stop or not selected_route:
        flash("Invalid route or stop selection.", "error")
        return redirect(url_for('dashboard'))
    
    # Find the current stop in the route sequence
    current_route_stop = RouteStop.query.filter_by(route_id=route_id, stop_id=current_stop_id).first()
    
    if not current_route_stop:
        flash("Selected stop is not on this route.", "error")
        return redirect(url_for('show_nearest_stop', latitude=user_lat, longitude=user_lon))
    
    # Get all stops in the route in sequence
    route_stops = RouteStop.query.filter(
        RouteStop.route_id == route_id
    ).order_by(RouteStop.stop_sequence).all()
    
    # Calculate route path for map display
    route_points = []
    stops_list = []
    
    for rs in route_stops:
        stop = Stop.query.get(rs.stop_id)
        
        # Add stop info for display
        stops_list.append({
            'stop_id': stop.stop_id,
            'name': stop.name,
            'sequence': rs.stop_sequence
        })
        
        # Add coordinates for map
        route_points.append({'lat': stop.latitude, 'lng': stop.longitude})
    
    # Find next departure time from the schedule
    now = datetime.now()
    current_time = now.time()
    current_day = now.strftime('%A')
    
    # Simplify day of week for query
    day_type = 'Weekday' if current_day in ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday'] else 'Weekend'
    
    # Get next scheduled departure for this route
    next_departure = Schedule.query.filter(
        Schedule.route_id == route_id,
        Schedule.is_active == True,
        Schedule.day_of_week == day_type,
        Schedule.start_time >= current_time
    ).order_by(Schedule.start_time).first()
    
    # If no more departures today, get the first one tomorrow
    if not next_departure:
        next_departure = Schedule.query.filter(
            Schedule.route_id == route_id,
            Schedule.is_active == True,
            Schedule.day_of_week == day_type
        ).order_by(Schedule.start_time).first()
    
    # Calculate arrival time at the current stop
    if next_departure:
        bus = Vehicle.query.get(next_departure.vehicle_id)
        departure_datetime = datetime.combine(
            now.date() if next_departure.start_time >= current_time else now.date() + timedelta(days=1),
            next_departure.start_time
        )
        
        # Calculate time from start of route to current stop
        route_start = RouteStop.query.filter_by(route_id=route_id).order_by(RouteStop.stop_sequence).first()
        stops_to_current = RouteStop.query.filter(
            RouteStop.route_id == route_id,
            RouteStop.stop_sequence >= route_start.stop_sequence,
            RouteStop.stop_sequence <= current_route_stop.stop_sequence
        ).order_by(RouteStop.stop_sequence).all()
        
        # Calculate distance from start to current stop
        distance_to_current = 0
        speed_kmh = 20  # km/h
        
        for i in range(len(stops_to_current) - 1):
            stop1 = Stop.query.get(stops_to_current[i].stop_id)
            stop2 = Stop.query.get(stops_to_current[i+1].stop_id)
            
            # Use predefined distance if available, otherwise calculate
            if stops_to_current[i+1].distance_from_previous:
                distance_to_current += stops_to_current[i+1].distance_from_previous
            else:
                distance_to_current += haversine_distance(
                    stop1.latitude, stop1.longitude,
                    stop2.latitude, stop2.longitude
                )
        
        # Calculate time to reach current stop (in minutes)
        travel_time_to_current = (distance_to_current / speed_kmh) * 60
        
        # Calculate arrival time at current stop
        arrival_time = departure_datetime + timedelta(minutes=travel_time_to_current)
        formatted_arrival = arrival_time.strftime('%I:%M %p')
        
        # Calculate time until arrival
        time_until_arrival = (arrival_time - now).total_seconds() / 60  # minutes
        if time_until_arrival < 0:
            arrival_message = "Next bus has already passed"
        elif time_until_arrival < 1:
            arrival_message = "Arriving now"
        else:
            arrival_message = f"{formatted_arrival} (in {int(time_until_arrival)} minutes)"
    else:
        arrival_message = "No scheduled departures found"
        bus = None
    
    # Check for any current delays
    live_tracking = LiveTracking.query.filter_by(route_id=route_id).order_by(LiveTracking.timestamp.desc()).first()
    delay_minutes = 0
    
    if live_tracking and live_tracking.is_delayed:
        delay_minutes = live_tracking.delay_minutes
        if arrival_message != "No scheduled departures found" and arrival_message != "Next bus has already passed":
            arrival_message += f" - Delayed by {delay_minutes} minutes"
    
    # Prepare ETA info
    eta_info = {
        'route_id': route_id,
        'route_name': selected_route.name,
        'route_color': selected_route.color_code,
        'bus_name': bus.name if bus else "Unknown",
        'arrival_time': arrival_message,
        'delay_minutes': delay_minutes,
        'route_points': route_points,
        'stops_list': stops_list
    }
    
    # Get the available routes using the helper function
    available_routes = get_available_routes(current_stop.stop_id)
    
    # Pass back to the nearest_stop template with ETA info
    return render_template('nearest_stop.html',
        stop_id=current_stop.stop_id,
        user_lat=user_lat,
        user_lon=user_lon,
        stop_lat=current_stop.latitude,
        stop_lon=current_stop.longitude,
        distance=haversine_distance(user_lat, user_lon, current_stop.latitude, current_stop.longitude),
        stop_name=current_stop.name,
        available_routes=available_routes,
        eta_info=eta_info,
        google_maps_api_key=app.config['GOOGLE_MAPS_API_KEY']
    )


def get_available_routes(stop_id):
    """Helper function to get available routes for a stop"""
    route_stops = RouteStop.query.filter_by(stop_id=stop_id).all()
    route_ids = [rs.route_id for rs in route_stops]
    routes = Route.query.filter(Route.route_id.in_(route_ids), Route.is_active==True).all()
    
    available_routes = []
    for route in routes:
        # Get all stops for this route in sequence order
        route_stops_seq = RouteStop.query.filter_by(route_id=route.route_id).order_by(RouteStop.stop_sequence).all()
        
        # Create a list of stops with sequence info
        stops_list = []
        for rs in route_stops_seq:
            stop = Stop.query.get(rs.stop_id)
            stops_list.append({
                'stop_id': stop.stop_id,
                'name': stop.name,
                'sequence': rs.stop_sequence,
                'latitude': stop.latitude,
                'longitude': stop.longitude
            })
        
        available_routes.append({
            'route_id': route.route_id,
            'name': route.name,
            'description': route.description,
            'color_code': route.color_code,
            'stops_json': json.dumps(stops_list)
        })
    
    return available_routes



@app.route('/bus_locations')
@login_required
def bus_locations():
    # Fetch current bus locations
    locations = LiveTracking.query.all()
    return jsonify([{
        'vehicle_id': loc.vehicle_id,
        'current_latitude': loc.current_latitude,
        'current_longitude': loc.current_longitude,
        'timestamp': loc.timestamp.isoformat(),
        'speed': loc.speed
    } for loc in locations])


@app.route('/stops')
@login_required
def get_stops():
    # Fetch all bus stops
    stops = Stop.query.all()
    return jsonify([{
        'stop_id': stop.stop_id,
        'name': stop.name,
        'latitude': stop.latitude,
        'longitude': stop.longitude
    } for stop in stops])



@app.route('/view_routes')
@login_required
def view_routes():
    """
    Render a page showing all bus routes
    """
    # Query to get all routes
    query = text("""
        SELECT route_id, name, description, color_code, is_active 
        FROM routes 
        ORDER BY route_id
    """)
    routes = db.session.execute(query).fetchall()
    
    return render_template('routes.html', routes=routes)

@app.route('/route_stops/<int:route_id>')
@login_required
def get_route_stops(route_id):
    """
    Get all stops for a specific route
    """
    query = text("""
        SELECT 
            rs.stop_sequence, 
            s.name, 
            s.latitude, 
            s.longitude, 
            rs.distance_from_previous
        FROM route_stops rs
        JOIN stops s ON rs.stop_id = s.stop_id
        WHERE rs.route_id = :route_id
        ORDER BY rs.stop_sequence
    """)
    stops = db.session.execute(query, {'route_id': route_id}).fetchall()
    
    # Convert query results to list of dictionaries
    stops_list = [
        {
            'stop_sequence': stop.stop_sequence,
            'name': stop.name,
            'latitude': stop.latitude,
            'longitude': stop.longitude,
            'distance_from_previous': stop.distance_from_previous
        } for stop in stops
    ]
    
    return jsonify(stops_list)

@app.route('/view_schedules')
@login_required
def view_schedules():
    """
    Render a page showing all bus schedules
    """
    # Query to get schedules with route and vehicle names
    query = text("""
        SELECT 
            s.schedule_id, 
            r.name AS route_name, 
            v.name AS vehicle_name, 
            s.start_time, 
            s.day_of_week, 
            s.is_active
        FROM schedule s
        JOIN routes r ON s.route_id = r.route_id
        JOIN vehicles v ON s.vehicle_id = v.vehicle_id
        ORDER BY s.day_of_week, s.start_time
    """)
    schedules = db.session.execute(query).fetchall()
    
    return render_template('schedules.html', schedules=schedules)

@app.route('/image')
def image_page():
    return render_template('route_chart_img.html')

@app.route('/serve-image')
def serve_image():
    return send_file('static/route_chart.jpg', mimetype='image/jpeg')
# Replace the existing WebSocket related code in your app.py
# Global variable to store mobile location


mobile_location = {'latitude': None, 'longitude': None}

@app.route('/mobile')
def mobile():
    return render_template('mobile.html')

@app.route('/update_location', methods=['POST'])
def update_location():
    global mobile_location
    data = request.get_json()
    
    if not data or 'latitude' not in data or 'longitude' not in data:
        return jsonify({'status': 'error', 'message': 'Invalid location data'}), 400
    
    mobile_location['latitude'] = data['latitude']
    mobile_location['longitude'] = data['longitude']
    
    print(f"Location updated: {mobile_location}")  # Debug print
    
    # Emit the new location to all connected clients
    socketio.emit('location_update', mobile_location)
    
    return jsonify({'status': 'Location updated successfully'}), 200

@app.route('/track')
def track():
    return render_template('track.html')

@socketio.on('connect')
def handle_connect():
    print("Client connected to WebSocket")  # Debug print
    socketio.emit('connection_established', {'status': 'connected'})
    
    # Send current location if available
    global mobile_location
    if mobile_location['latitude'] is not None and mobile_location['longitude'] is not None:
        socketio.emit('location_update', mobile_location)




# feedba
@app.route('/feedback', methods=['GET', 'POST'])
def submit_feedback():
    if request.method == 'POST':
        user_id = request.form.get('user_id', 999)  # Default user ID or get from form
        feedback_type = request.form.get('feedback_type')
        description = request.form.get('description')
        rating = request.form.get('rating')
        
        feedback = Feedback(
            user_id=user_id,
            feedback_type=feedback_type,
            description=description,
            rating=rating
        )
        
        db.session.add(feedback)
        db.session.commit()
        
        flash('Your feedback has been submitted successfully!', 'success')
        return redirect(url_for('thank_you'))
    
    return render_template('feedback.html')
@app.route('/thank-you')
def thank_you():
    return render_template('thank_you.html')

@app.route('/admin/feedbacks')
def admin_feedbacks():
    feedbacks = Feedback.query.all()
    return render_template('admin_feedbacks.html', feedbacks=feedbacks)

@app.route('/admin/resolve/<int:feedback_id>', methods=['POST'])
def resolve_feedback(feedback_id):
    feedback = Feedback.query.get_or_404(feedback_id)
    feedback.is_resolved = True
    db.session.commit()
    
    flash('Feedback has been marked as resolved', 'success')
    return redirect(url_for('admin_feedbacks'))


# Admin Routes
@app.route('/admin/update_bus_location', methods=['POST'])
@login_required
def update_bus_location():
    # Endpoint for updating bus location (could be used by mobile app)
    if not current_user.is_admin:
        return jsonify({'error': 'Unauthorized'}), 403

    data = request.json
    vehicle_id = data.get('vehicle_id')
    latitude = data.get('latitude')
    longitude = data.get('longitude')
    speed = data.get('speed', 0)

    # Create or update live tracking
    existing_track = LiveTracking.query.filter_by(vehicle_id=vehicle_id).first()
    
    if existing_track:
        existing_track.current_latitude = latitude
        existing_track.current_longitude = longitude
        existing_track.speed = speed
        existing_track.timestamp = datetime.now(pytz.utc)
    else:
        new_track = LiveTracking(
            vehicle_id=vehicle_id,
            current_latitude=latitude,
            current_longitude=longitude,
            speed=speed
        )
        db.session.add(new_track)

    db.session.commit()
    return jsonify({'status': 'success'})

# Error Handlers
@app.errorhandler(404)
def page_not_found(e):
    return render_template('404.html'), 404

@app.errorhandler(500)
def internal_server_error(e):
    return render_template('500.html'), 500

# Initialize Database
def init_db():
    with app.app_context():
        db.create_all()

# # Run the Application
# if __name__ == '__main__':
#     try:
#         init_db()
#         print("Starting the server...")
#         socketio.run(app, debug=True, host='0.0.0.0', port=5000)
#     except Exception as e:
#         print(f"Error starting server: {e}")
