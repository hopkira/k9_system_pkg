import datetime
import pickle
import os.path
from googleapiclient.discovery import build
from google.auth.transport.requests import Request
from ament_index_python.packages import get_package_share_directory

SCOPES = ['https://www.googleapis.com/auth/calendar.readonly']

def get_calendar_service():
    creds = None
    pkg_share = get_package_share_directory('k9_system_pkg')
    token_file = os.path.join(pkg_share, 'assets', 'token.pickle')
    if os.path.exists(token_file):
        with open(token_file, 'rb') as token:
            creds = pickle.load(token)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            raise RuntimeError("No valid credentials. Run the setup script.")
    
    return build('calendar', 'v3', credentials=creds)

def get_upcoming_events(max_results=5):
    service = get_calendar_service()
    now = datetime.datetime.utcnow().isoformat() + 'Z'  # UTC time
    events_result = service.events().list(
        calendarId='primary', timeMin=now,
        maxResults=max_results, singleEvents=True,
        orderBy='startTime').execute()
    
    events = events_result.get('items', [])
    return [(e['summary'], e['start'].get('dateTime', e['start'].get('date')))
            for e in events]
