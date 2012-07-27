me.Action("destroyMovieLogin").Triggered.connect(destroyMovieLogin);
me.Action("destroyMoviePayment").Triggered.connect(destroyMoviePayment);
me.Action("ticketAdded").Triggered.connect(ticketAdded);

var tickets = 0;

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
    var movieDim = scene.GetEntityByName("MovieDim");
    if(movieDim)
        movieDim.Exec(1, "Hide");
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
    var movieDim = scene.GetEntityByName("MovieDim");
    if(movieDim)
        movieDim.Exec(1, "Hide");
}

function DelayedDestroyMoviePayment()
{

    if (scene.EntityByName("MoviePaymentDialog"))
    {
        scene.EntityByName("MoviePaymentDialog").Exec(1, "Cleanup");
        scene.RemoveEntity(scene.EntityByName("MoviePaymentDialog").id);
    }
}

function ticketAdded(entity_name)
{
    tickets++;
    var ent = scene.EntityByName(entity_name);
    print(entity_name);
    ent.Exec(1, "SetTicketCount", tickets);
}
