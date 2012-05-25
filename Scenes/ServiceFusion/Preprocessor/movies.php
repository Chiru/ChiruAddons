<?php

//define('HIDE_ADVERTISE',TRUE);
//define("RDFAPI_INCLUDE_DIR", "C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
define("RDFAPI_INCLUDE_DIR", "C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/rdfapi-php/api/");
//require('C:/Program Files (x86)/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
require('C:/Program Files/Apache Software Foundation/Apache2.2/htdocs/magpierss/rss_fetch.inc');
include(RDFAPI_INCLUDE_DIR . "RdfAPI.php");

$shows = array();
$current_show = null;
$last_entity = "";
$root_element = "SHOW";
$child_enities = array("ORIGINALTITLE", "THEATREAUDITORIUM", "DTTMSHOWSTART");

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

        parse_movies($appNode, $dataSourceUrl, $resStm);
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

function parse_movies($moviesDocResource, $dataSourceUrl, $dataSrcStm)
{
    global $shows;
    
    $model1 = ModelFactory::getDefaultModel();
    $model1->addNamespace("cie", "http://cie/");
    
    $time = date('Y-m-d\TH:i:sP');
    
    $moviesDoc = $moviesDocResource;
    
    if (!empty($dataSourceUrl))
    {
        $data = new Resource ("http://cie/data");
        $metaData = new Resource ("http://cie/metadata");
        
        $xml_url = $dataSourceUrl;
        $ch = curl_init($xml_url);
        curl_setopt($ch, CURLOPT_HEADER, false);
        curl_setopt($ch, CURLOPT_RETURNTRANSFER, true);
        $result = curl_exec($ch);
        if ($result == false)
        {
            header("HTTP/1.0 400 Bad Request");
            return;
        }
        
        $xml_parser = xml_parser_create();
        xml_set_element_handler($xml_parser, "StartElement", "EndElement");
        xml_set_character_data_handler($xml_parser , "ReadCharData");
        xml_parser_set_option($xml_parser, XML_OPTION_TARGET_ENCODING, "UTF-8");

        if (!xml_parse($xml_parser, $result, true)) {
            die(sprintf("XML error: %s at line %d",
                        xml_error_string(xml_get_error_code($xml_parser)),
                        xml_get_current_line_number($xml_parser)));
        }
        xml_parser_free($xml_parser);
        
        for($i = 0; $i < count($shows); $i++)
        {
            $movie = $shows[$i];
            if(isset($movie))
            {
                if (isset($movie['ORIGINALTITLE']) && isset($movie['THEATREAUDITORIUM']) && isset($movie['THEATREAUDITORIUM']))
                {
                    $statement = new Statement ($moviesDoc, $data, new Literal ($movie['ORIGINALTITLE'] . ';' . $movie['THEATREAUDITORIUM'] . ';' . $movie['DTTMSHOWSTART']));
                    $model1->add($statement);
                }
            }
        }
        
        $model1->add($dataSrcStm);
        
        $source = new Resource ("http://cie/source-application");
        $statement1 = new Statement ($moviesDocResource, $source, new Literal ('movies'));
        $model1->add($statement1);
        
        $geo = new Resource ("http://cie/geo");
        $statement3 = new Statement ($moviesDocResource, $geo, new Literal ('oulu'));
        $model1->add($statement3);
            
        $timeRes = new Resource ("http://cie/datetime");
        $timeLit = new Literal ($time);
        $timeLit->setDatatype('http://www.w3.org/2001/XMLSchema#dateTime');
        $statement4 = new Statement ($moviesDocResource, $timeRes, $timeLit);
        $model1->add($statement4);

        $rdfString = $model1->writeRdfToString();
        $model1->close();

        echo utf8_decode($rdfString);
    }
}

// Callback function for xml parser.
function StartElement($parser, $name, $attrs) 
{
    global $current_show, $last_entity, $child_enities, $root_element;
    if($name == $root_element)
        $current_show = array();
    else if(is_array($current_show) && in_array($name, $child_enities))
        $last_entity = $name;
}

// Callback function for xml parser.
function EndElement($parser, $name) 
{
    global $current_show, $shows, $last_entity, $root_element;
    if ($name == $root_element && is_array($current_show))
    {
        array_push($shows, $current_show);
        $current_show = null;
        $last_entity = "";
    }
    if (!empty($last_entity))
        $last_entity = "";
}

// Callback function for xml parser.
function ReadCharData($parser, $data)
{
    global $last_entity, $current_show;
    if (!empty($last_entity))
    {
        if(isset($current_show[$last_entity]))
            $current_show[$last_entity] = $current_show[$last_entity] . (string)$data;
        else
            $current_show[$last_entity] = $data;
    }
}
?>