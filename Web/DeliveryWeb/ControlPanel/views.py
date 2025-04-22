from django.http import HttpResponse, HttpRequest
from django.template import loader
# Create your views here.
from django.conf import settings



def panel(request: HttpRequest) -> HttpResponse:
    template = loader.get_template('panel.html')
    return HttpResponse(template.render({'BOT_ADDRESS': ''}, request))