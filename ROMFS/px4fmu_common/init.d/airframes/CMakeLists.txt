############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(
	# [0-999] Reserved (historical)"

	# [1000, 1999] Simulation setups"
	
	# [2000, 2999] Standard planes"

	# [3000, 3999] Flying wing"

	# [4000, 4999] Quadrotor x"
	4101_quad_x_hils
	4102_f450_quad_x_hils
	4103_Devcopter_quad_x_hils

	# [5000, 5999] Quadrotor +"
	5101_quad_+_hils

	# [6000, 6999] Hexarotor x"
	6101_hexa_x_hils

	# [7000, 7999] Hexarotor +"
	7101_hexa_+_hils

	# [8000, 8999] Octorotor +"
	8101_octo_x_hils

	# [9000, 9999] Octorotor +"
	9101_octo_+_hils

	# [10000, 10999] Quadrotor Wide arm / H frame"

	# [11000, 11999] Hexa Cox

	# [12000, 12999] Octo Cox

	# [13000, 13999] VTOL

	# [14000, 14999] Tri Y

)
