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
	
	//$dataSrc = new Resource ($dataSourceUrl);
	//$statement = new Statement ($newsDoc, "http://cie/data-source", new Literal ($dataSourceUrl));
	
	
	if (!empty($dataSourceUrl))
	{
		//$listItem = new BlankNode("item");
		$data = new Resource ("http://cie/data");
		$metaData = new Resource ("http://cie/metadata");
		//$statement = new Statement ($newsDoc, $data, $listItem);
		//$model1->add($statement);
			
		$rss = fetch_rss($dataSourceUrl);
		$count = count($rss->items);
		// get 3 first items from rss feed
		for ($i = 0; $i < min($count, 3); $i++) {
			$item = $rss->items[$i];
			$title = $item['title'];
			
			$statement = new Statement ($newsDoc, $data, new Literal ($title));
			$model1->add($statement);
		
			list($firstWord) = explode(' ', $title); 
			
			$statement = new Statement ($newsDoc, $metaData, new Literal ($firstWord));
			$model1->add($statement);
		}
	}
	else
	{
		$data = new Resource ("http://cie/data");
		
		$title = new Resource ("http://cie/title");
		$statement = new Statement ($data, $title, new Literal ('Tepco uses camera to survey Fukushima plant'));
		$model1->add($statement);
		
		$metaData = new Resource ("http://cie/metadata");
		$statement = new Statement ($title, $metaData, new Literal ('fukushima'));
		$model1->add($statement);
		$statement = new Statement ($title, $metaData, new Literal ('tepco'));
		$model1->add($statement);
		
		$text = new Resource ("http://cie/text1");
		$statement = new Statement ($data, $text, new Literal ('(TEPCO)\'s tsunami-crippled '));
		$model1->add($statement);
		
		//$statement = new Statement ($text, $metaData, new Literal ('TEPCO'));
		//$model1->add($statement);
		
		$text = new Resource ("http://cie/text2");
		$statement = new Statement ($data, $text, new Literal ('Fukushima'));
		$model1->add($statement);
		
		$statement = new Statement ($text, $metaData, new Literal ('fukushima'));
		$model1->add($statement);
		
		$text = new Resource ("http://cie/text3");
		$statement = new Statement ($data, $text, new Literal (' Daiichi Nuclear Power Plant. The operators of the tsunami-stricken Fukushima nuclear power plant said Thursday they had ...'));
		$model1->add($statement);
		
	//	$statement = new Statement ($text, $metaData, new Literal ('fukushima'));
	//	$model1->add($statement);
	//	$statement = new Statement ($text, $metaData, new Literal ('tsunami'));
	//	$model1->add($statement);
	}


	$model1->add($dataSrcStm);
	
	$source = new Resource ("http://cie/source-application");
	$statement1 = new Statement ($newsDoc, $source, new Literal ('news'));
	$model1->add($statement1);
	
	$geo = new Resource ("http://cie/geo");
	$statement3 = new Statement ($newsDoc, $geo, new Literal ('fukushima'));
	$model1->add($statement3);
	
	$geo = new Resource ("http://cie/geo");
	$statement3 = new Statement ($newsDoc, $geo, new Literal ('-29.36,3.49,20.26'));
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

