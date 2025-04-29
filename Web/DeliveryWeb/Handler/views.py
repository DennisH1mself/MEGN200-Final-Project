from django.shortcuts import render
from django.http import HttpRequest, HttpResponse
from django.views.decorators.csrf import csrf_exempt
import json
import requests
from django.conf import settings
# Create your views here.
@csrf_exempt
def setMotorSpeed(request: HttpRequest):
    print('MOTOR SPEED RECIEVED')
    if request.method == 'POST':
        # Get the speed from the request body
        speed = 255 if int(json.loads(request.body).get('motor', 0)) == 1 else 0
        try:
            # Send the speed to the bot's motor control endpoint
            requests.post(settings.BOT_ADDRESS + '/control/motor', json={'motor': speed})
        except:
            pass
        return HttpResponse(status=200)
    else:
        return HttpResponse("Invalid request method", status=405)
    """if request.method == 'POST':
        # Get the speed from the request body
        speed = int(json.loads(request.body).get('motor', 0))
        try:
            print('ATTEMPTING SEND')
            requests.post(settings.BOT_ADDRESS + '/control/motor', json={'motor': speed}, timeout=3)
        except:
            pass
        return HttpResponse(status=200)
    else:
        return HttpResponse("Invalid request method", status=405)"""
    
@csrf_exempt
def setServoAngle(request: HttpRequest):
    if request.method == 'POST':
        # Get the angle from the request body
        angle = int(json.loads(request.body).get('servo', 0))
        try:
            # Send the angle to the bot's servo control endpoint
            requests.post(settings.BOT_ADDRESS + '/control/servo', json={'servo': angle})
        except:
            pass
        return HttpResponse(status=200)
    else:
        return HttpResponse("Invalid request method", status=405)
@csrf_exempt
def getLocation(request: HttpRequest):
    if request.method == 'GET':
        # Get the distance from the request body
        # try:
        """response = requests.get(settings.BOT_ADDRESS + '/control/location', timeout=3, headers={'Accept-Encoding': 'identity'}, stream=True)
        response.raw.decode_content = True
        data = json.loads(response.raw.read(decode_content=True))
        print(data)"""
        try:
            response = requests.get(settings.BOT_ADDRESS + '/control/location', timeout=3)
            data = response.json()
        except requests.exceptions.ConnectTimeout as e:
            # Handle the timeout error
            data = {
                'latitude': -1,
                'longitude': -1,
            }
            return HttpResponse(json.dumps(data), content_type='application/json', headers={'Content-Length': str(len(json.dumps(data)))})
        # Check if the request was successful
        if response.status_code == 200:
            # Read the response data
            # Return the data as a JSON response
            return HttpResponse(json.dumps(data), content_type='application/json', headers={'Content-Length': str(len(json.dumps(data)))})
        else:
            # Handle the error case
            return HttpResponse("Error while getting distance", status=500)
        
        # return HttpResponse(response.json(), content_type='application/json')
        """except:
            return HttpResponse("Error while getting distance", status=500)"""
        
    else:
        return HttpResponse("Invalid request method", status=405)