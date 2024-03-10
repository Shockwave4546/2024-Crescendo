package frc.robot.utils;

import java.util.Objects;

@FunctionalInterface
public interface TriConsumer<S, T, U> {
  void accept(S s, T t, U u);

  default TriConsumer<S, T, U> andThen(final TriConsumer<? super S, ? super T, ? super U> after) {
    Objects.requireNonNull(after);

    return (s, t, u) -> {
      accept(s, t, u);
      after.accept(s, t, u);
    };
  }
}