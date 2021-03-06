package com.pervasive.helloairsim;

import java.util.concurrent.Callable;

public interface CustomCallable<R> extends Callable<R> {
    void postExecute(R result);
    void preExecute();
}
