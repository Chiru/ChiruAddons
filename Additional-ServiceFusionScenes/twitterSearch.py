import json
import urllib2
import time
from datetime import datetime

class TwitterSearch:

    def __init__(self):
        #self.tweetArray = []
        self.url = ""

    def getUserTweets(self, userID, userName, numTweets):

        if(userID != ""):
            self.url = "http://api.twitter.com/1/statuses/user_timeline.json?user_id=%s&count=%s&trim_user=false&include_rts=true&include_entities=true" % (userID, numTweets) 
        elif(userName != ""):
            self.url = "http://api.twitter.com/1/statuses/user_timeline.json?screen_name=%s&count=%s&trim_user=false&include_rts=true&include_entities=true" % (userName, numTweets)
        else:
            print 'Give user id or user name'
            return
        
        return self.makeRequest(self.url)

    def getHashTagTweets(self, hashTag, searchType, numTweets, language):

        if(not(searchType == "mixed" or searchType == "recent" or searchType == "popular")):
            print 'searchType: %s is not valid. SearchType must be: mixed, recent or popular'
            return

        else:
            self.url = "http://search.twitter.com/search.json?q=%%23%s&result_type=%s&rpp=%s&lang=%s&include_rts=false&include_entities=true" % (hashTag, searchType, numTweets, language)
            return self.makeRequest(self.url)

    def makeRequest(self, url):
        tweetArray = []
        print "making request: %s" % (url)
        try:
            response = urllib2.urlopen(url)
            userTweets = json.loads(response.read())
            #print userTweets
            userTweets = userTweets['results'] if 'results' in userTweets else userTweets
            for t in userTweets:
                tweet = Tweet(t)
                tweetArray.append(tweet)
 
        except Exception as e:
            print 'Could not get tweets: %s' % (e)

        return tweetArray

class Tweet():

    def __init__(self, tw):
        self.user = ""
        self.userImgUrl = ""
        self.id = ""
        self.location = ""
        self.retweetCount = 0
        self.date = ""
        self.text = ""
        self.parseTweet(tw)

    def __str__(self):
        output = 'Tweet content:\nid: %s\nuser: %s\nuser image url: %s\nlocation: %s\ndate: %s\ntext: %s\nretweets: %s\n' % (self.id, self.user, self.userImgUrl, self.location, self.date, self.text, self.retweetCount)
        return output.encode("utf-8")

    def returnAsJson(self):
        output = "{'id': %s, 'user': %s, 'user_image_url': %s, 'location': %s, 'date': %s, 'text': %s, 'retweets': %s}" % (self.id, self.user, self.userImgUrl, self.location, self.date, self.text, self.retweetCount)
        return output.encode("utf-8")

    def parseTweet(self, jsonTweet):
        self.id = jsonTweet['id'] if 'id' in jsonTweet else "" 
        self.user = (jsonTweet['user']['name'] if 'user' in jsonTweet else "") or (jsonTweet['from_user'] if 'from_user' in jsonTweet else "")
        self.location = jsonTweet['user']['location'] if 'user' in jsonTweet else ""
        self.text = jsonTweet['text'] if 'text' in jsonTweet else ""
        self.retweetCount = (jsonTweet['retweet_count'] if 'retweet_count' in jsonTweet else 0) or (jsonTweet['metadata']['recent_retweets'] if ('metadata' in jsonTweet and 'recent_retweets' in jsonTweet['metadata']) else 0)
        self.date = jsonTweet['created_at'] if 'created_at' in jsonTweet else ""
        self.userImgUrl = (jsonTweet['user']['profile_image_url'] if 'user' in jsonTweet else "") or (jsonTweet['profile_image_url'] if 'profile_image_url' in jsonTweet else "")
        #self.date = self.convertDate(self.date)

    def convertDate(self, date):
        time_struct = time.strptime(date, "%a %b %d %H:%M:%S +0000 %Y")
        return datetime.fromtimestamp(time.mktime(time_struct))

    def getUser():
        return self.user

    def getID():
        return self.id

    def getLocation():
        return self.location

    def getReTweetCount():
        return self.retweetCount

    def getDate():
        return self.date

    def getText():
        return self.text

#class User:
