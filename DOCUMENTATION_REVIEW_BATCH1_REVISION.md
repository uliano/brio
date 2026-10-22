# Review of the revised first documentation batch

Reviewed [PR 2](https://github.com/uliano/brio/pull/2) at commit
`2f7771e3e598655a6830587f43e90c1945c9f583`, against its base
`d8ba686a44a2d4a3d2bdccaa648c3ced57f9517d`.

This is author feedback, not public documentation. No GitHub comment or
review was submitted. No implementation or PR content was changed.

## Assessment

The revision addresses the substantial first-review findings. The learning
route is now a useful model for the next batches. Do not restart it or add
another broad rewrite cycle. Make the small corrections below, retain the
explicit local GUI verification handoff, and continue with the planned work.

In particular:

- The fresh-checkout clangd instructions identify the release and host
  databases and explain that they do not follow the active debug preset.
- Host-test instructions configure, build, then run, including the short
  reference instructions and the root README.
- Scheduling now describes fixed selection precedence and its accepted
  limits without promising fairness or a universal response-time bound.
- Blocking calls and application protocol progress are distinguished from
  post's non-waiting behavior.
- Event size accounts for padding, and the tutorial quotes the checked
  four-byte variant rather than a general largest-payload-plus-one formula.
- Timer conversion no longer promises a physical minimum delay merely
  because its integer arithmetic rounds upward.
- Debugger observations distinguish nonzero queue bookkeeping from overflow,
  and ordinary WFI sleep from the separate DBG_STOP concern.
- The GUI guide describes present coverage without deciding that missing
  graphical operations must remain absent forever.
- VS Code and terminal procedures are alternatives in the navigation.
- The reference map is substantially easier to scan. The queue's internal
  comment now defines its terms and preserves its invariants in a readable
  progression. This is more than adding a friendly preface.

## One technical correction: the exact half-range boundary

Location: [time_event.hpp, deadline limits](https://github.com/uliano/brio/blob/2f7771e3e598655a6830587f43e90c1945c9f583/brio/kernel/time_event.hpp#L54)
and its arm() comment at line 103.

The new comment permits delays and periods up to and including 2^31 ticks.
The author's examination of process() at that exact boundary is correct,
but the timer API also exposes ticks_to_next() and next_deadline(). Those
use the signed difference in the opposite direction. Exactly half a counter
range cannot be interpreted consistently as future by both comparisons.

I compiled and ran a probe against the pinned PR headers. With now() zero,
one timer due at tick 100, and another armed for 0x80000000 ticks:

```text
ticks_to_next() = 0
next_deadline() = 2147483648
events posted by process() = 0
```

The distance query says something is already due, and the nearest-deadline
query chooses the distant timer instead of the one at tick 100, even though
process() posts neither. Changing the distant delay to 0x7fffffff makes both
queries correctly select tick 100.

For the public timer contract, state that supported delays and periods are
strictly less than 2^31 ticks. Keep the separate overdue-distance restriction.
If the exact-boundary behavior of process() is retained as an explanation,
identify it as an implementation observation, not a supported interval for
all timer operations. No code change is requested.

The limits stated in the progress record should match the final contract;
its historical account currently alternates between exclusive and inclusive.
The public teaching path needs no further expansion about this edge case.

## Two small wording refinements

### Scope the non-concurrency rule to one core

[README.md, execution model](https://github.com/uliano/brio/blob/2f7771e3e598655a6830587f43e90c1945c9f583/README.md#L24)
and [the private-state explanation](https://github.com/uliano/brio/blob/2f7771e3e598655a6830587f43e90c1945c9f583/README.md#L108)
still state that handlers never run concurrently without the single-core
qualification. The introduction eventually states its scope, but the root
README also advertises two kernels on two cores.

Add "on the same core" to the relevant general statements, or introduce the
whole model as the behavior of one kernel on one core. This is a small
precision improvement, not a request for an early multicore tutorial.

### Include sustainable traffic in the short-handler advice

[introduction.md, accepted limits](https://github.com/uliano/brio/blob/2f7771e3e598655a6830587f43e90c1945c9f583/docs/learn/introduction.md#L180)
now correctly explains starvation, but the final advice mentions only short
handlers and queues sized for bursts. Include the requirement that producers
must not keep earlier queues continually busy. Queue capacity absorbs bursts;
it does not fix sustained overload. One sentence is sufficient.

## Verification

The local checkout changed branch while the review was in progress. To
avoid mixing revisions, final checks were repeated on a complete temporary
archive of the pinned PR commit, without changing the user's checkout.

- Existing prose checker: 19 files, zero errors, zero review items.
- All ten kernel headers: compiler-based comment stripping and normalized
  comparison against 4d20f8f confirmed token identity.
- Relative Markdown destinations and heading anchors: 147 checked, no
  unresolved destination found by the lightweight check.
- The exact-half-range probe above was compiled and run against that pinned
  archive, using the existing host compiler. It is a temporary review probe,
  not a new application or a committed test.
- Compared the revised setup instructions with the checked-in presets,
  .clangd fragments, VS Code settings and launch configuration.
- No board upload, hardware debug session, interactive VS Code validation,
  full image gate, or full host test suite was performed in this review.
  Code-token identity and documentation-only changes do not establish those
  interactive results; the handoff remains applicable.

## Guidance for continuing

Carry this writing pattern into subsequent batches. Preserve the distinction
between reader comprehension and technical fact checking. For exact limits,
check the cooperating operations, not just the function nearest the new
sentence: the timer boundary is an example of why that matters.

Avoid turning every clarification into another long warning in the tutorial.
Make the few precise corrections, keep detailed contracts in the reference,
and let the reader continue toward the task. Missing demonstration
applications remain future work, as already agreed.
