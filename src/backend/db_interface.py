import sqlite3
import json
from datetime import datetime
from pathlib import Path


class RoverDB:
    """
    Database interface for logging Mars Rover navigation and sensor data to SQLite.
    
    This class provides methods to store and retrieve rover telemetry including:
    - Navigation data (position, velocity, heading)
    - Sensor readings (temperature, battery, obstacle detection)
    - Mission timestamps and status
    """
    
    def __init__(self, db_path="rover_data.db"):
        """
        Initialize the RoverDB connection.
        
        Args:
            db_path (str): Path to the SQLite database file
        """
        self.db_path = db_path
        self.conn = None
        self.cursor = None
        self._connect()
        self._create_tables()
    
    def _connect(self):
        """Establish connection to the SQLite database."""
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
    
    def _create_tables(self):
        """Create tables for navigation and sensor data if they don't exist."""
        # Navigation data table
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS navigation (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                position_x REAL,
                position_y REAL,
                position_z REAL,
                velocity_x REAL,
                velocity_y REAL,
                velocity_z REAL,
                heading REAL,
                status TEXT
            )
        """)
        
        # Sensor data table
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS sensor_data (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                temperature REAL,
                battery_level REAL,
                obstacle_detected BOOLEAN,
                distance_to_obstacle REAL,
                sensor_metadata TEXT
            )
        """)
        
        self.conn.commit()
    
    def log_navigation(self, position, velocity, heading, status="active"):
        """
        Log navigation data to the database.
        
        Args:
            position (tuple): (x, y, z) coordinates
            velocity (tuple): (vx, vy, vz) velocity components
            heading (float): Heading angle in degrees
            status (str): Rover status (e.g., 'active', 'paused', 'blackout')
        
        Returns:
            int: ID of the inserted record
        """
        timestamp = datetime.now().isoformat()
        
        self.cursor.execute("""
            INSERT INTO navigation 
            (timestamp, position_x, position_y, position_z, 
             velocity_x, velocity_y, velocity_z, heading, status)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            timestamp,
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            heading,
            status
        ))
        
        self.conn.commit()
        return self.cursor.lastrowid
    
    def log_sensor_data(self, temperature, battery_level, 
                       obstacle_detected, distance_to_obstacle=None, 
                       metadata=None):
        """
        Log sensor data to the database.
        
        Args:
            temperature (float): Temperature reading in Celsius
            battery_level (float): Battery level percentage (0-100)
            obstacle_detected (bool): Whether an obstacle was detected
            distance_to_obstacle (float, optional): Distance to obstacle in meters
            metadata (dict, optional): Additional sensor metadata
        
        Returns:
            int: ID of the inserted record
        """
        timestamp = datetime.now().isoformat()
        metadata_json = json.dumps(metadata) if metadata else None
        
        self.cursor.execute("""
            INSERT INTO sensor_data 
            (timestamp, temperature, battery_level, obstacle_detected, 
             distance_to_obstacle, sensor_metadata)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (
            timestamp,
            temperature,
            battery_level,
            obstacle_detected,
            distance_to_obstacle,
            metadata_json
        ))
        
        self.conn.commit()
        return self.cursor.lastrowid
    
    def get_recent_navigation(self, limit=10):
        """
        Retrieve recent navigation records.
        
        Args:
            limit (int): Maximum number of records to retrieve
        
        Returns:
            list: List of navigation records as dictionaries
        """
        self.cursor.execute("""
            SELECT * FROM navigation 
            ORDER BY timestamp DESC 
            LIMIT ?
        """, (limit,))
        
        columns = [desc[0] for desc in self.cursor.description]
        return [dict(zip(columns, row)) for row in self.cursor.fetchall()]
    
    def get_recent_sensor_data(self, limit=10):
        """
        Retrieve recent sensor data records.
        
        Args:
            limit (int): Maximum number of records to retrieve
        
        Returns:
            list: List of sensor data records as dictionaries
        """
        self.cursor.execute("""
            SELECT * FROM sensor_data 
            ORDER BY timestamp DESC 
            LIMIT ?
        """, (limit,))
        
        columns = [desc[0] for desc in self.cursor.description]
        return [dict(zip(columns, row)) for row in self.cursor.fetchall()]
    
    def get_stats(self):
        """
        Get summary statistics from the database.
        
        Returns:
            dict: Dictionary containing record counts and other statistics
        """
        nav_count = self.cursor.execute(
            "SELECT COUNT(*) FROM navigation"
        ).fetchone()[0]
        
        sensor_count = self.cursor.execute(
            "SELECT COUNT(*) FROM sensor_data"
        ).fetchone()[0]
        
        return {
            "navigation_records": nav_count,
            "sensor_records": sensor_count,
            "database_path": self.db_path
        }
    
    def close(self):
        """Close the database connection."""
        if self.conn:
            self.conn.close()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
