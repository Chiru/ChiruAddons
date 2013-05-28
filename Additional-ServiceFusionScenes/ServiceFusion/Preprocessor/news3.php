<?php
define('HIDE_ADVERTISE',TRUE);
//define("RDFAPI_INCLUDE_DIR", "C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
define("RDFAPI_INCLUDE_DIR", "C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
//require('C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
require('C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
include(RDFAPI_INCLUDE_DIR . "RdfAPI.php");

if (isset($_POST['data']))
{
	$inRDFData = $_POST['data'];

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
		
		do_http_search($appNode, $dataSourceUrl, $resStm);
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

function do_http_search($newsDocResource, $dataSourceUrl, $dataSrcStm)
{
	$model1 = ModelFactory::getDefaultModel();
	$model1->addNamespace("cie", "http://cie/");

	$time = date('Y-m-d\TH:i:sP');
	$dataCnt = '';
	
	$newsDoc = $newsDocResource;

	
	if (!empty($dataSourceUrl))
	{
		$data = new Resource ("http://cie/data");
		$metaData = new Resource ("http://cie/metadata");

			
		$rss = fetch_rss($dataSourceUrl);
		$count = count($rss->items);
		// get 3 first items from rss feed
		for ($i = 0; $i < min($count, 3); $i++) {
			$item = $rss->items[$i];
			$title = $item['title'];
			$description = $item['description'];
			
			mb_detect_encoding($title, "UTF-8", true) == "UTF-8" ? : $title = utf8_encode($title);
			mb_detect_encoding($description, "UTF-8", true) == "UTF-8" ? : $description = utf8_encode($description);
			
			$statement = new Statement ($newsDoc, $data, new Literal ($title));
			$model1->add($statement);
		
			list($firstWord) = explode(' ', $title); 
			
			$statement = new Statement ($newsDoc, $metaData, new Literal ($firstWord));
			$model1->add($statement);
			
			$statement = new Statement ($newsDoc, $data, new Literal ($description));
			$model1->add($statement);
		}
	}

	$model1->add($dataSrcStm);
	
	$source = new Resource ("http://cie/source-application");
	$statement1 = new Statement ($newsDoc, $source, new Literal ('news'));
	$model1->add($statement1);
	
	$geo = new Resource ("http://cie/geo");
	$statement3 = new Statement ($newsDoc, $geo, new Literal ('oulu'));
	$model1->add($statement3);
		
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

