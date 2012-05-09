<?php

define('YAHOO_API_ID', 'KNZfYq4q');

function convert_gps_coordinates($gps) {
    $result = "0,0,0";

    $gpsCoords = array_map('trim', explode(',', $gps));
    if (count($gpsCoords) == 2)
    {
            $phi = deg2rad(floatval($gpsCoords[0]));
            $theta = deg2rad(floatval($gpsCoords[1]));

            // globe distortion fix
            $phi *= 0.9746;
            $theta += deg2rad(143.76);

            // local coordinates relative to the globe
            $gx = 35.8 * cos($phi) * sin($theta);
            $gy = 35.8 * cos($phi) * cos($theta);
            $gz = 35.95 * sin($phi);

            $result = strval($gx) . "," . strval($gy) . "," . strval($gz);
    }

    return $result;
}

// Address can contain infomation of city and country aswell.
// You can find more infomation what can be in address section in
// "http://developer.yahoo.com/geo/placefinder/" address.
// e.g. "13 Wall street New York USA"
function get_address_gps_cordinates($address)
{
    if (isset($address))
    {
        $request_url = "http://where.yahooapis.com/geocode?q=" . $address . '&flags=CJ&appid=' . YAHOO_API_ID;
        $request_url = str_replace(' ', '+', $request_url);

        $ch = curl_init($request_url);
        curl_setopt($ch, CURLOPT_HEADER, false);
        curl_setopt($ch, CURLOPT_RETURNTRANSFER, true);
        $response = curl_exec($ch);
        
        if ($response == false)
            return false;
        else
        {
            $response = json_decode($response, true);
            // Assume that first result is the right one.
            if (isset($response['ResultSet']['Results']))
            {
                $response = $response['ResultSet']['Results'][0];
                return $response['latitude'] . ',' . $response['longitude'];
            }
            else
                return false;
        }
    }
    else
    {
        return false;
    }
}

?>