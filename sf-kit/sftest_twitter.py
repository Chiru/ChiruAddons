import sfkit
import logging, json, urllib
import time, datetime

logging.basicConfig(format='%(asctime).19s %(message)s', level=logging.DEBUG)
logger = logging.getLogger("sf.twitter")
logger.setLevel(logging.DEBUG)

def main():
    logger.info("starting")
    sfkit.open_db()
    scene_cnx = sfkit.open_scene()
    service_name = "twitter"

    sfkit.add_source(fetch_callback = fetch_feed,
                     data_format = 'title+text',
                     refresh_period = 5*60,
                     name = service_name)

    # continue doing periodic data refresh
    logger.info("entering serve loop")
    sfkit.serve(scene_cnx, service_name)
    logger.info("finished")

def toutf8(s):
    if isinstance(s, unicode):
        return s.encode('utf-8')
    else:
        return s

def hashtag_tweets(tag, result_type="recent", language=""):
    hashtag_url = "http://search.twitter.com/search.json"
    count = 5
    query = { 'q': '#' + toutf8(tag),
              'result_type': result_type,
              'rpp': count,
              'lang': language,
              'include_entities': 'true',
              'include_rts': 'false',
              'include_entities': 'true'}

    # let IOError propagate
    f = urllib.urlopen(hashtag_url + '?' + urllib.urlencode(query))
    data = f.read()
    z = json.loads(data)
    print 'first z', z[0]
    return map(Tweet, z)

def user_tweets(username, limit=5):
    timeline_url = "http://api.twitter.com/1/statuses/user_timeline.json"
    query = { 'screen_name': toutf8(username),
              'count': limit,
              'trim_user': 'true',
              'include_rts': 'true',
              'include_entities': 'true' }
    # let IOError propagate
    f = urllib.urlopen(timeline_url + '?' + urllib.urlencode(query))
    data = f.read()
    z = json.loads(data)
    print 'first z', z[0]
    print 'before decoding', data[:1000]
    return map(Tweet, z)

# 
class MyOpener(urllib.FancyURLopener):
    version = "sfkit_twitter/0.1"

# documented interface despite the underscore
urllib._urlopener = MyOpener()

class Tweet(object):
    def __init__(self, d):
        self.id = d.get('id', u'')
        screen_name = real_name = None
        try:
            screen_name = d['user']['screen_name']
            real_name = d['user']['name']
        except KeyError:
            print "no userinfo:", d.get('user')
            from_user = d.get('from_user', u'<unknown>')

        self.screen_name = screen_name or from_user
        self.real_name = real_name or from_user

        self.text = d.get('text', u'')
        x = d.get('created_at')
        if x:
            time_struct = time.strptime(x, "%a %b %d %H:%M:%S +0000 %Y")
            self.created_at = datetime.datetime.fromtimestamp(time.mktime(time_struct))
        else:
            self.created_at = None
        

        ## unused
        # metadata = d.get('metadata', {})
        # recent_retweets = metadata.get('recent_retweets', 0)
        # self.retweet_count = d.get('retweet_count', recent_retweets)
        # self.date = d.get('created_at', u'')

def convertDate(self, date):
    time_struct = time.strptime(date, "%a %b %d %H:%M:%S +0000 %Y")
    return datetime.fromtimestamp(time.mktime(time_struct))

def fetch_feed(query=None):
    if not query:
        logger.debug("query unset so not fetching")
        return None

    try:
        tweets = user_tweets(query)
    except IOError, what:
        logger.error("didn't get user_tweets: " + str(what))
        return None

    entries_text = []
    for t in tweets[:4]:
        entries_text.append(u'%s\n  %s\n' % (query, t.text))

    return dict(title=u'Tweets from ' + query, entries=entries_text)

if __name__ == '__main__':
    main()
