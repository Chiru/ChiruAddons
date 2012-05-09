<?php
/**
 * Simple function to replicate PHP 5 behaviour
 */
function microtime_float()
{
    list($usec, $sec) = explode(" ", microtime());
    return ((float)$usec + (float)$sec);
}

$a = 'dsdsd';
$b = 'dsdsds';
$c = 'asdasdas';
//for ($j = 0; $j > 1; $j++)
$time_start = microtime_float();

for ($i = 0; $i < 1000000; ++$i)
{
    $var = '';
    // OPTION 1
    //$var = "The value of variable A is $a. The value of variable B is $b. The value of variable C is $c.";

    // OPTION 2
    //$var = 'The value of variable A is '.$a.'. The value of variable B is '.$b.'. The value of variable C is '.$c.'.';
}

$time_end = microtime_float();
$time = $time_end - $time_start;

echo "Did nothing in $time seconds\n";
?>