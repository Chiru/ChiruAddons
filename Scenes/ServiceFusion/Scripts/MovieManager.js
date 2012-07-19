me.Action("destroyMovieLogin").Triggered.connect(destroyMovieLogin);
me.Action("destroyMoviePayment").Triggered.connect(destroyMoviePayment);

function destroyMovieLogin()
{
    // This function is invoked from movieLogin when payment procedure is cancelled.

    //Destroy all movie entities
    if (scene.EntityByName("cart_item"))
        scene.EntityByName("cart_item").placeable.visible = false;

    frame.DelayedExecute(0.2).Triggered.connect(DelayedDestroyMovieLogin);

    if (scene.EntityByName("MoviePaymentDialog"))
    {
        scene.EntityByName("MoviePaymentDialog").Exec(1, "Cleanup");
        scene.RemoveEntity(scene.EntityByName("MoviePaymentDialog").id);
    }
    if (scene.EntityByName("MovieSeatDialog"))
        scene.RemoveEntity(scene.EntityByName("MovieSeatDialog").id);

    if (scene.EntityByName("MovieLoginDialog"))
    {
        var loginEnt = scene.EntityByName("MovieLoginDialog");
        loginEnt.placeable.visible = false;
    }
}

function DelayedDestroyMovieLogin()
{
    if (scene.EntityByName("MovieLoginDialog"))
    {
        scene.EntityByName("MovieLoginDialog").Exec(1, "Cleanup");
        scene.RemoveEntity(scene.EntityByName("MovieLoginDialog").id);
    }
}

function destroyMoviePayment()
{
    // This function is invoked from moviePayment when payment procedure is finished.

    //Destroy all movie entities
    if (scene.EntityByName("cart_item"))
        scene.EntityByName("cart_item").placeable.visible = false;

    frame.DelayedExecute(0.2).Triggered.connect(DelayedDestroyMoviePayment);
    if (scene.EntityByName("MoviePaymentDialog"))
    {
        var paymentEnt = scene.EntityByName("MoviePaymentDialog");
        paymentEnt.placeable.visible = false;
    }

    if (scene.EntityByName("MovieSeatDialog"))
        scene.RemoveEntity(scene.EntityByName("MovieSeatDialog").id);


    if (scene.EntityByName("MovieLoginDialog"))
    {
        scene.EntityByName("MovieLoginDialog").Exec(1, "Cleanup");
        scene.RemoveEntity(scene.EntityByName("MovieLoginDialog").id);
    }
}

function DelayedDestroyMoviePayment()
{

    if (scene.EntityByName("MoviePaymentDialog"))
    {
        scene.EntityByName("MoviePaymentDialog").Exec(1, "Cleanup");
        scene.RemoveEntity(scene.EntityByName("MoviePaymentDialog").id);
    }
}
