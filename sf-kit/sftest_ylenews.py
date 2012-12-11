import sfkit
import feedparser

def main():
    sfkit.open_db()
    scene_cnx = sfkit.open_scene()
    service_name = "yle-news"

    sfkit.add_source(fetch_callback = fetch_feed,
                     data_format = 'title+text',
                     refresh_period = 5*60,
                     name = service_name)

    # continue doing periodic data refresh
    sfkit.serve(scene_cnx, service_name)

def fetch_feed(query=None):
    if not query:
        query = 'news'
    yle_rss_url = 'http://yle.fi/uutiset/rss/uutiset.rss?osasto=' + query
    yle_rss = feedparser.parse(yle_rss_url)
    entries_text = []
    for e in yle_rss.entries[:4]:
        entries_text.append(u'%s\n  %s\n' % (e.title, e.summary))

    return dict(title=yle_rss.feed.title, entries=entries_text)

if __name__ == '__main__':
    main()
