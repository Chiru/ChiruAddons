<?php
define('MAGPIE_CACHE_AGE', 30);
//define("RDFAPI_INCLUDE_DIR", "C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
define("RDFAPI_INCLUDE_DIR", "C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
//require('C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
require('C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
require('cordinate.php');
include(RDFAPI_INCLUDE_DIR . "RdfAPI.php");
if (!defined('HIDE_ADVERTISE')) define('HIDE_ADVERTISE',TRUE);

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
		// get 6 items from rss feed
		$numEvents = 0;
                
                $locations = array();
                
		for ($i = 0 ; $i < $count ; $i++)
		{
			if ($numEvents == 6)
				break;
		
			$item = $rss->items[$i];
			$title = $item['title'];
			$description = $item['description'];
			
			mb_detect_encoding($title, "UTF-8", true) == "UTF-8" ? : $title = utf8_encode($title);
			mb_detect_encoding($description, "UTF-8", true) == "UTF-8" ? : $description = utf8_encode($description);
			
			$lines = explode("\n", $description);
			for ($j = 0 ; $j < count($lines) - 2 ; $j++)
			{
				if (strrpos($lines[$j], "Paikka:") !== false)
				{
					$locationLine = $lines[$j + 2];
					$locArr = explode(",", $locationLine);
					$location = trim($locArr[count($locArr) - 1]);
					$cityLoc = strrpos($location, "(");
					$cityEndLoc = strrpos($location, ")", $cityLoc);
					if ($cityLoc !== false && $cityEndLoc !== false)
					{
						$city = substr($location, $cityLoc + 1, $cityEndLoc - $cityLoc - 1);
						if (strcmp($city, "Oulu") == 0)
						{
							$numEvents++;
							$statement = new Statement ($newsDoc, $data, new Literal ($title));
							$model1->add($statement);
			
                                                        $statement = new Statement ($newsDoc, $data, new Literal ($location));
                                                        $model1->add($statement);
                                                        
                                                        $location = str_replace('(', ' ', $location);
                                                        $location = str_replace(')', ' ', $location);
                                                        array_push($locations, $location);
                                                        /*$gps_cords   = get_address_gps_cordinates($location);
                                                        //$local_cords = convert_gps_coordinates($gps_cords);
                                                        
                                                        //$statement = new Statement ($newsDoc, $data, new Literal ($local_cords));
                                                        //$model1->add($statement);
                                                        if ($gps_cords != false)
                                                        {
                                                            $statement = new Statement ($newsDoc, $data, new Literal ($gps_cords));
                                                            $model1->add($statement);
                                                        }
                                                        else
                                                        {
                                                            $statement = new Statement ($newsDoc, $data, new Literal ("Failed"));
                                                            $model1->add($statement);
                                                        }*/
							
							break;
						}
					}
				}
			}

		
			//list($firstWord) = explode(' ', $title); 
			
			//$statement = new Statement ($newsDoc, $metaData, new Literal ($firstWord));
			//$model1->add($statement);
			
			//$statement = new Statement ($newsDoc, $data, new Literal ($description));
			//$model1->add($statement);
		}
	}

	$model1->add($dataSrcStm);
	
	$source = new Resource ("http://cie/source-application");
	$statement1 = new Statement ($newsDoc, $source, new Literal ('news'));
	$model1->add($statement1);
	
        if (count($locations) > 0)
        {
            $geo = new Resource ("http://cie/geo");
            foreach($locations as $v)
            {
                $gps_cords = get_address_gps_cordinates($v);
                
                // Check if Yahoo PlaceFinder found a given address.
                if($gps_cords != false)
                {
                    $statement = new Statement ($newsDoc, $geo, new Literal ($gps_cords));
                    $model1->add($statement);
                    
                    $statement = new Statement ($newsDoc, $geo, new Literal (convert_gps_coordinates($gps_cords)));//'-29.36,3.49,20.26'));//
                    $model1->add($statement);
                }
            }
        }
        else
        {
            $geo = new Resource ("http://cie/geo");
            $statement3 = new Statement ($newsDoc, $geo, new Literal ('oulu'));
            $model1->add($statement3);   
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

