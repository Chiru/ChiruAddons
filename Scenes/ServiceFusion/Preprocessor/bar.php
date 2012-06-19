<?php
define('HIDE_ADVERTISE',TRUE);
//define("RDFAPI_INCLUDE_DIR", "C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
define("RDFAPI_INCLUDE_DIR", "C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
//require('C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
require('C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
require('cordinate.php');
include(RDFAPI_INCLUDE_DIR . "RdfAPI.php");

$inRDFData = null;
if (isset($_POST['data']))
    $inRDFData = $_POST['data'];
else if (isset($_GET['data']))
    $inRDFData = $_GET['data'];
    
if (!is_null($inRDFData))
{
	$postDataModel = ModelFactory::getDefaultModel();
	$postDataModel->loadFromString($inRDFData, 'xml');
	$inDataSrc = new Resource("http://cie/data-source");
	$resStm = $postDataModel->findFirstMatchingStatement(null, $inDataSrc, NULL);
	if ($resStm != NULL)
	{
            $dataSourceUrl = $resStm->getLabelObject();
            $dataSourceUrl = urldecode($dataSourceUrl);
            $dataSourceUrl = trim($dataSourceUrl);
            $appNode = $resStm->getSubject();

            fetch_song($appNode, $dataSourceUrl, $resStm);
	}
	else
	{
            header("HTTP/1.0 400 Bad Request");
	}
}
else
{
    header("HTTP/1.0 400 Bad Request");
}

function fetch_song($newsDocResource, $dataSourceUrl, $dataSrcStm)
{
	$model1 = ModelFactory::getDefaultModel();
	$model1->addNamespace("cie", "http://cie/");

	$time = date('Y-m-d\TH:i:sP');
	$dataCnt = '';
	
	$newsDoc = $newsDocResource;
	$model1->add($dataSrcStm);
	
	$source = new Resource ("http://cie/source-application");
	$statement1 = new Statement ($newsDoc, $source, new Literal ('music'));
	$model1->add($statement1);
	
	if (!empty($dataSourceUrl))
	{
            $data = new Resource ("http://cie/data");
            $metaData = new Resource ("http://cie/metadata");

            $rss = fetch_rss($dataSourceUrl);
            $current = "";

            foreach ($rss->items as $item ) {
                    $itemTitle = $item['title'];
                    //echo $itemTitle . $item[description];
                    if (strcmp($itemTitle, "Current") == 0)
                            $current = $item['description'];
            }

            $title = $rss->channel['title'];

            $location = $rss->channel['location'];
            $address = $rss->channel['address'];


            $statement = new Statement ($newsDoc, $data, new Literal ($title));
            $model1->add($statement);
            $statement = new Statement ($newsDoc, $metaData, new Literal ('title'));
            $model1->add($statement);

            $statement = new Statement ($newsDoc, $data, new Literal ($current));
            $model1->add($statement);
            $statement = new Statement ($newsDoc, $metaData, new Literal ('current'));
            $model1->add($statement);

            $geo = new Resource ("http://cie/geo");
            $statement = new Statement ($newsDoc, $geo, new Literal (convert_gps_coordinates($location)));
            $model1->add($statement);
            $statement = new Statement ($newsDoc, $geo, new Literal ($address));
            $model1->add($statement);
	}
	
	
	$timeRes = new Resource ("http://cie/datetime");
	$timeLit = new Literal ($time);
	$timeLit->setDatatype('http://www.w3.org/2001/XMLSchema#dateTime');
	$statement4 = new Statement ($newsDoc, $timeRes, $timeLit);
	$model1->add($statement4);

	$rdfString = $model1->writeRdfToString();
	$model1->close();

	echo $rdfString;
}

?> 

