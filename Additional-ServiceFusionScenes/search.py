import tundra as tundra
import twitterSearch
#import googlePlusSearch
#import sfkit

class TwitterService:
    def __init__(self):
        tundra.LogInfo("** Starting Twitter srvice **");
        self.twitter_search = twitterSearch.TwitterSearch()
        self.previousSearch = ""

        assert tundra.Scene().connect("SceneAdded(QString)", self.sceneAdded)

#    def registerTwitterSeach():
#        sfkit.open_db()
#        scene = sfkit.open_scene()
#        service_name = "twitter-search"
#
#        sfkit.add_source(fetch_callback = twitterSearch,
#                         data_format = 'user+location+date+text+retweetTimes',
#                         refresh_period = 5*60,
#                         name=service_name)

    #sfkit.connect_source(scene, service_name)
    #sfkit.do_refresh(twitterSearch, service_name)
    #sfkit.serve_forever()

    def twitterSearch(self, searchType, language, numResults, searcWord):
        data=[]

        if(searchType == "user"):
            data = self.twitter_search.getUserTweets("", searcWord, numResults)

        elif(searchType=="content"):
            data = self.twitter_search.getHashTagTweets(searcWord, "recent", numResults, language)

        else:
            print 'Define searchType: user or content. Current searchType=%s' %(searchType)

        return data

    def sceneAdded(self, name):
        self.scene = tundra.Scene().GetSceneRaw(name)
        assert self.scene.connect("AttributeChanged(IComponent*, IAttribute*, AttributeChange::Type)", self.OnAttributeChanged)


    def OnAttributeChanged(self, component, attribute, changeType):
        entity = component.ParentEntity()
        if(entity.name != "Twitter"):
            return

        if(component.typeName != "EC_DynamicComponent"):
            return

        searchWord = component.GetAttribute("searchWord")
        searchType = component.GetAttribute("searchType")

        if(searchWord=="" or searchType==""):
            return

        if(searchWord == self.previousSearch):
            return

        results = self.twitterSearch(searchType, "", 5, searchWord)        
        self.previousSearch = searchWord

        if(results):
            if(not component.GetAttribute("result")):
                component.CreateAttribute("string", "result")
    
            result_str = ""
            #resultsJson = []

            for i in results:
                 result_str =result_str+"tweetContent:"+ i.returnAsJson()
                 #resultsJson.append(i.returnAsJson())

            #print result_str
            #print resultsJson
            component.SetAttribute("result",result_str)
            #component.SetAttribute("result",resultsJson)

#def googlePlusSearch(searchType, language, numResults, searcWord):

#    if(searchType == "user"):
#        data = googlePlus_search.getUserProfile(userID)

#    elif(searchType=="content"):
#        data = googlePlus_search.searchActivities(searcWord, numResults, language)

#    else:
#        print 'Define searchType: user or content'

#    return data


if __name__ == '__main__':
     search = TwitterService()
