import foursquare


if 1:
    creds = open("4sq-creds.txt").read().split()
    client = foursquare.Foursquare(client_id=creds[0], client_secret=creds[1], redirect_uri='http://chiru.cie.fi/no-such-thing')
    # Build the authorization url for your app
    auth_uri = client.oauth.auth_url()
    print auth_uri
else:
    access_token = open("4sq-token.txt").read().strip()


