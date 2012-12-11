import foursquare
import sfkit

MY_LOCATION = '65.06097202499998,25.468180775642395'

def main():
    print "starting"
    sfkit.open_db()
    scene_cnx = sfkit.open_scene()
    service_name = "twitter"

    creds = open("4sq-creds.txt").read().split()
    client = foursquare.Foursquare(client_id=creds[0], client_secret=creds[1], redirect_uri='http://chiru.cie.fi/no-such-thing')

    sfkit.add_source(fetch_callback = fetch_feed,
                     data_format = 'title+text',
                     refresh_period = 5*60,
                     name = service_name)

    # continue doing periodic data refresh
    print "entering serve loop"
    sfkit.serve(scene_cnx, service_name)
    print "finished"


def search_4sq(query):
    r = client.venues.search(params={'query': '', 'll': MY_LOCATION})
    
