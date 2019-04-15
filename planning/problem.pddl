(define (problem p01)
(:domain bookWorld)
(:objects
tbot3 - robot
tbot3_init_loc - location
book_1 book_2 - book
trolly_2 trolly_1 - bin
book_1_iloc book_2_iloc - location
trolly_2_iloc trolly_1_iloc - location
Operating_Systems_Architecture - subject
small large - size
)
(:init
(Book_At book_1 book_1_iloc)
(Book_At book_2 book_2_iloc)
(Bin_At trolly_2 trolly_2_iloc)
(Bin_At trolly_1 trolly_1_iloc)
(Book_Subject book_1 Operating_Systems_Architecture)
(Book_Size book_1 small)
(Book_Subject book_2 Operating_Systems_Architecture)
(Book_Size book_2 large)
(Bin_Subject trolly_2 Operating_Systems_Architecture)
(Bin_Size trolly_2 small)
(Bin_Subject trolly_1 Operating_Systems_Architecture)
(Bin_Size trolly_1 large)
(Robot_At tbot3 tbot3_init_loc)
(Empty_Basket tbot3)
)
(:goal ENTER YOUR GOAL FORMULA HERE )
)